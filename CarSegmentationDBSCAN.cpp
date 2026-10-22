#include <ogx/Plugins/EasyPlugin.h>
#include <ogx/Data/Clouds/CloudHelpers.h>
#include <ogx/Data/Clouds/SphericalSearchKernel.h>
#include <ogx/Math/Ransac.h>
#include <queue>
#include <unordered_map>
#include <limits>
#include <unordered_set>

#include "Utils.h"
#include "EasyMethodExtension.h"

using namespace ogx;

struct BoundingBox {
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();
};


struct CarSegmentationDBSCAN : public EasyMethodExtension
{
    Data::ResourceID _nodeId;
    float _roadDistanceThreshold = 0.2;
    int _ransacIterations = 100;
    float _maxCarHeight = 3.0f;
    float _clusterRadius = 0.4;
    float _minClusterSize = 1000;
    float _maxClusterSize = 7000;
    float _minPointsForCore = 9;
    float _minCarLength = 2.5f;
    float _maxCarLength = 10.5f;
    float _minCarWidth = 1.3f;
    float _maxCarWidth = 2.3f;
    float _minCarHeightDim = 1.0f;
    float _maxCarHeightDim = 1.6f;

    CarSegmentationDBSCAN() : EasyMethodExtension(L"Car segmentation in point cloud.")
    {
    }

    virtual void DefineParameters(ParameterBank& bank)
    {
        bank.Add(L"NodeId", _nodeId).AsNode();
        bank.Add(L"Ransac threshold", _roadDistanceThreshold);
        bank.Add(L"Ransac iterations", _ransacIterations);
        bank.Add(L"Max Car Height", _maxCarHeight);
        bank.Add(L"Cluster Radius", _clusterRadius);
        bank.Add(L"Minimum Cluster Size", _minClusterSize);
        bank.Add(L"Maximum Cluster Size", _maxClusterSize);
        bank.Add(L"PointsForCore", _minPointsForCore);
        bank.Add(L"Min Car Length", _minCarLength);
        bank.Add(L"Max Car Length", _maxCarLength);
        bank.Add(L"Min Car Width", _minCarWidth);
        bank.Add(L"Max Car Width", _maxCarWidth);
        bank.Add(L"Min Car Height Dim", _minCarHeightDim);
        bank.Add(L"Max Car Height Dim", _maxCarHeightDim);
    }
    /**
     * @brief Removes the ground plane from the point cloud using RANSAC.
     *
     * Fits a plane to the input points using RANSAC, then classifies each point
     * based on its distance from this plane. Points are considered car candidates
     * if they lie between the ground plane and the maximum car height.
     * Non-candidate points are marked as PS_DELETED in the cloud's state vector.
     *
     * @param pointsRange Range of points to process (will be modified - state flags updated).
     * @return Vector of booleans of size equal to pointsRange.size(), where true
     *         indicates the point is a car candidate.
     */
    std::vector<bool> RemoveGroundPlane(ogx::Data::Clouds::PointsRange& pointsRange)
    {
        std::vector<ogx::Math::Point3D> points;
        points.reserve(pointsRange.size());

        std::vector<bool> isValid(pointsRange.size(), false);

        for (const auto& point : ogx::Data::Clouds::RangeLocalXYZConst(pointsRange))
        {
            points.push_back(point.cast<double>());
        }

        auto ransac = ogx::Math::Plane3DRansac(_roadDistanceThreshold, _ransacIterations);
        ogx::Math::Plane3D roadPlane;
        ransac.findPlane3D(points, roadPlane);

        std::vector<ogx::Data::Clouds::State> stateVector;
        pointsRange.GetStates(stateVector);

        for (size_t i = 0; i < points.size(); i++)
        {
            auto distance = ogx::Math::CalcPointToPlaneDistance3D(points[i], roadPlane);
            if (distance >= _roadDistanceThreshold && distance <= _maxCarHeight)
            {
                isValid[i] = true;
            }
            else
            {
                stateVector[i].set(ogx::Data::Clouds::PS_DELETED);
            }
        }

        pointsRange.SetStates(stateVector);

        int validCount = 0;
        for (bool v : isValid) if (v) validCount++;
        OGX_LINE.Msg(ogx::Level::Info, L"Ground plane removed. Candidate points: " +
            std::to_wstring(validCount) + L" / " + std::to_wstring(isValid.size()));

        return isValid;
    }
    /**
     * @brief Clusters candidate points and filters clusters down to likely cars.
     *
     * Clustering uses BFS to discover connected groups of candidate points (a
     * DBSCAN-inspired region growing). A point propagates its cluster only if it has
     * at least _minPointsForCore neighbours within _clusterRadius (i.e. is a core
     * point in DBSCAN terminology). Non-core points can still join a cluster but do
     * not extend it further.
     *
     * After clustering, two filters are applied:
     *   1. Size filter: clusters with point counts outside the
     *      [_minClusterSize, _maxClusterSize] range are discarded.
     *   2. Dimension filter: for each surviving cluster an axis-aligned bounding box
     *      is computed, and the cluster is kept only if its dimensions match a car
     *      (length in [_minCarLength, _maxCarLength], width in [_minCarWidth,
     *      _maxCarWidth], height in [_minCarHeightDim, _maxCarHeightDim], where Z is
     *      assumed to be the vertical axis).
     *
     * Final cluster IDs are written to the "ClusterID" layer and all points not
     * belonging to a kept cluster are marked as PS_DELETED.
     *
     * @param context Plugin context (used for progress bar updates).
     * @param cloud Pointer to the point cloud (used for spherical neighbour search).
     * @param allPointsRange Range of all points in the cloud (used for reading positions
     *                       and writing the final ClusterID layer).
     * @param isValid Vector indicating which points are candidates for clustering
     *                (output from RemoveGroundPlane).
     */
    void ClusterPoints(Context& context, ogx::Data::Clouds::ICloud* cloud, ogx::Data::Clouds::PointsRange& allPointsRange, const std::vector<bool>& isValid)
    {
        std::vector<ogx::Data::Clouds::Point3D> points;
        allPointsRange.GetXYZ(points);

        std::vector<int> clusterIds(allPointsRange.size(), 0);
        int nextClusterId = 0;

        auto pointIndexLayers = cloud->FindLayers(L"PointIndex");
        auto pointIndexLayer = pointIndexLayers[0];

        for (size_t i = 0; i < allPointsRange.size(); i++) {
            if (i % 10000 == 0) {
                if (!context.Feedback().Update(static_cast<float>(i) / allPointsRange.size())) {
                    ReportError(L"Couldn't update progress bar.");
                }
            }
            if (isValid[i] && clusterIds[i] == 0) {
                nextClusterId++;
                std::queue<int> myQueue;
                myQueue.push(i);
                while (!myQueue.empty()) {
                    int first = myQueue.front();
                    myQueue.pop();
                    if (clusterIds[first] == 0) {
                        clusterIds[first] = nextClusterId;
                        auto sphericalSearchKernel = ogx::Data::Clouds::SphericalSearchKernel(
                            ogx::Math::Sphere3D(_clusterRadius, points[first].cast<double>())
                        );
                        ogx::Data::Clouds::PointsRange sphericalRange;
                        cloud->GetAccess().FindPoints(sphericalSearchKernel, sphericalRange);
                        std::vector<ogx::StoredReal> neighbourIndices;
                        sphericalRange.GetLayerVals(neighbourIndices, *pointIndexLayer);
                        if (neighbourIndices.size() >= _minPointsForCore) {
                            for (size_t j = 0; j < neighbourIndices.size(); j++) {
                                int idx = static_cast<int>(neighbourIndices[j]);
                                if (isValid[idx] && clusterIds[idx] == 0) {
                                    myQueue.push(idx);
                                }
                            }
                        }

                    }
                }
            }
        }
        std::unordered_map<int, int> clusterSizes;
        for (size_t i = 0; i < clusterIds.size(); i++) {
            if (clusterIds[i] != 0) {
                clusterSizes[clusterIds[i]]++;
            }

        }
        OGX_LINE.Msg(ogx::Level::Info, L"Initial clusters: " + std::to_wstring(nextClusterId));
        for (size_t i = 0; i < clusterIds.size(); i++) {
            if (clusterIds[i] != 0) {
                int size = clusterSizes[clusterIds[i]];
                if (size < _minClusterSize || size > _maxClusterSize) {
                    clusterIds[i] = 0;
                }
            }

        }
        std::unordered_map<int, BoundingBox> clusterBoxes;
        for (size_t i = 0; i < clusterIds.size(); i++) {
            if (clusterIds[i] != 0) {
                int id = clusterIds[i];
                const auto& p = points[i];

                if (p.x() < clusterBoxes[id].minX) clusterBoxes[id].minX = p.x();
                if (p.x() > clusterBoxes[id].maxX) clusterBoxes[id].maxX = p.x();
                if (p.y() < clusterBoxes[id].minY) clusterBoxes[id].minY = p.y();
                if (p.y() > clusterBoxes[id].maxY) clusterBoxes[id].maxY = p.y();
                if (p.z() < clusterBoxes[id].minZ) clusterBoxes[id].minZ = p.z();
                if (p.z() > clusterBoxes[id].maxZ) clusterBoxes[id].maxZ = p.z();
            }
        }

        for (size_t i = 0; i < clusterIds.size(); i++) {
            if (clusterIds[i] != 0) {
                const auto& box = clusterBoxes[clusterIds[i]];
                float dimX = box.maxX - box.minX;
                float dimY = box.maxY - box.minY;
                float dimZ = box.maxZ - box.minZ;

                float length = std::max(dimX, dimY);
                float width = std::min(dimX, dimY);
                float height = dimZ;

                bool isCar =
                    length >= _minCarLength && length <= _maxCarLength &&
                    width >= _minCarWidth && width <= _maxCarWidth &&
                    height >= _minCarHeightDim && height <= _maxCarHeightDim;

                if (!isCar) {
                    clusterIds[i] = 0;
                }
            }
        }

        std::unordered_set<int> remainingIds;
        for (int cid : clusterIds) {
            if (cid != 0) remainingIds.insert(cid);
        }
        OGX_LINE.Msg(ogx::Level::Info, L"Clusters surviving dimension filter: " + std::to_wstring(remainingIds.size()));

        ogx::String layerName = L"ClusterID";
        auto const layers = cloud->FindLayers(layerName);
        auto clusterLayer = layers.empty() ? cloud->CreateLayer(layerName, 0.0) : layers[0];
        std::vector<float> clusterIdsFloat;
        clusterIdsFloat.reserve(clusterIds.size());
        for (size_t i = 0; i < clusterIds.size(); i++) {
            clusterIdsFloat.push_back(static_cast<float>(clusterIds[i]));
        }
        allPointsRange.SetLayerVals(clusterIdsFloat, *clusterLayer);

        std::vector<ogx::Data::Clouds::State> stateVector;
        allPointsRange.GetStates(stateVector);
        for (size_t i = 0; i < clusterIds.size(); i++) {
            if (clusterIds[i] == 0) {
                stateVector[i].set(ogx::Data::Clouds::PS_DELETED);
            }
        }
        allPointsRange.SetStates(stateVector);
    }
    /**
     * @brief Main entry point of the car segmentation plugin.
     *
     * Pipeline:
     *   1. Validates input parameters.
     *   2. Duplicates the input cloud node to preserve the original.
     *   3. Creates a "PointIndex" layer storing each point's index (0..N-1) so that
     *      neighbours returned by spherical search can be mapped back to local arrays.
     *   4. Removes the ground plane (RANSAC).
     *   5. Clusters the remaining candidate points using DBSCAN algorithm.
     *
     * @param context Plugin context provided by the framework.
     */
    virtual void Run(Context& context)
    {
        OGX_LINE.Msg(ogx::Level::Info, L"CarSegmentationDBSCAN started.");

        if (_roadDistanceThreshold <= 0.0f) {
            ReportError(L"Ransac threshold must be greater than zero!");
        }
        if (_ransacIterations <= 0) {
            ReportError(L"Ransac iterations must be greater than zero!");
        }
        if (_maxCarHeight <= 0.0f) {
            ReportError(L"Max car height must be greater than zero!");
        }
        if (_clusterRadius <= 0.0f) {
            ReportError(L"Cluster radius must be greater than zero!");
        }
        if (_minPointsForCore < 1.0f) {
            ReportError(L"Points for core must be at least 1!");
        }
        if (_minClusterSize < 1.0f) {
            ReportError(L"Minimum cluster size must be at least 1!");
        }
        if (_maxClusterSize <= 0.0f) {
            ReportError(L"Maximum cluster size must be greater than zero!");
        }
        if (_minClusterSize > _maxClusterSize) {
            ReportError(L"Minimum cluster size cannot be greater than maximum cluster size!");
        }

        auto node = context.m_project->TransTreeFindNode(_nodeId);
        if (!node) {
            ReportError(L"Node not found! Please select a valid point cloud.");
        }
        auto newNode = DuplicateNode(context.Project(), node, node->GetParent());
        auto cloud = GetCloud(context, newNode);

        ogx::Data::Clouds::PointsRange pointsRange;
        cloud->GetAccess().GetAllPoints(pointsRange);

        std::vector<float> pointIndices;
        pointIndices.reserve(pointsRange.size());

        auto pointIndexLayer = cloud->CreateLayer(L"PointIndex", 0.0);

        for (size_t i = 0; i < pointsRange.size(); i++) {
            pointIndices.push_back(i);
        }

        pointsRange.SetLayerVals(pointIndices, *pointIndexLayer);

        auto isValid = RemoveGroundPlane(pointsRange);

        ClusterPoints(context, cloud, pointsRange, isValid);

        OGX_LINE.Msg(ogx::Level::Info, L"CarSegmentationDBSCAN finished.");
    }
};

OGX_EXPORT_METHOD(CarSegmentationDBSCAN)