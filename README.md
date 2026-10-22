# 3D Point Cloud Car Segmentation (C++)

> **Uwaga techniczna:** Projekt został zrealizowany jako wtyczka (plugin) do zamkniętego, akademickiego frameworka przetwarzania chmur punktów (z wykorzystaniem interfejsu FRAMES / biblioteki `ogx`). Repozytorium służy jako demonstracja implementacji przestrzennych algorytmów w C++. Kod udostępniam w celach poglądowych.

Projekt to wysokowydajny algorytm napisany w C++, służący do automatycznej detekcji i segmentacji pojazdów z przestrzennych chmur punktów scen ulicznych. Rozwiązanie opiera się wyłącznie na kryteriach geometrycznych i jest w pełni interpretowalne.

## Rezultaty segmentacji

Algorytm osiągnął wysoką skuteczność, poprawnie identyfikując **11 z 12 pojazdów** widocznych w badanej chmurze punktów:
* **9 pojazdów** zostało perfekcyjnie wyizolowanych jako pojedyncze obiekty.
* **1 klaster** poprawnie zlokalizował samochody, ale scalił dwa zaparkowane bezpośrednio obok siebie pojazdy (długość klastra ~10m).
* **1 pojazd** nie został wykryty z powodu zbyt rzadkiej reprezentacji punktowej w surowej chmurze (niespełnienie kryteriów wymiarowych).

<p align="center">
  <img width="400" alt="Chmura przed segmentacją" src="https://github.com/user-attachments/assets/4751f534-88b6-447e-8928-55292e9ac41b"/>
  <img width="400" alt="Wyizolowane pojazdy" src="https://github.com/user-attachments/assets/1bdfdbaf-c9f0-4a65-bb0e-bb756a84f8dc" />
  <br>
  <i>Widok surowej chmury punktów (lewo) oraz wyizolowanych klastrów reprezentujących pojazdy (prawo).</i>
</p>

## Architektura potoku przetwarzania (Pipeline)

Algorytm realizuje segmentację w pięciu sekwencyjnych etapach:
1. **Modelowanie podłoża (RANSAC):** Dopasowanie płaszczyzny do punktów drogi.
2. **Klasyfikacja kandydatów:** Odrzucenie punktów należących do gruntu oraz obiektów znajdujących się powyżej maksymalnej wysokości pojazdu.
3. **Klastrowanie przestrzenne (DBSCAN):** Grupowanie pozostałych punktów metodą *region growing* (BFS + jądro sferyczne).
4. **Filtracja rozmiaru (Point Count):** Wstępne odrzucenie klastrów o zbyt małej lub zbyt dużej liczbie punktów.
5. **Filtracja wolumetryczna (Bounding Box):** Rygorystyczna weryfikacja fizycznych wymiarów (długość, szerokość, wysokość) wyznaczonego obszaru ograniczającego dla każdego klastra.

## Analiza ograniczeń (Wnioski i Future Work)

W trakcie eksperymentów i ewaluacji zidentyfikowano następujące wyzwania algorytmiczne:

* **Niejednorodna gęstość chmury:** Wzajemne przesłanianie się obiektów (occlusion) i kąt padania wiązki skanera powodują skrajne różnice w gęstości. Wdrożenie filtra Bounding Box znacząco ustabilizowało detekcję względem samego liczenia punktów, jednak skrajnie niepełne reprezentacje (np. sam bok karoserii) nadal bywają odrzucane.
* **Under-segmentation (Sklejanie obiektów):** Klasyczny DBSCAN nie radzi sobie z rozdzielaniem pojazdów zaparkowanych tuż obok siebie, co prowadzi do łączenia ich w jeden nadmiarowy klaster.
* **Fałszywe detekcje (Brak cech semantycznych):** Opieranie się wyłącznie na geometrii sprawia, że algorytm może pomylić gęstą grupę pieszych z samochodem. Zaostrzenie limitu wysokości Bounding Boxa niweluje ten błąd, ale podnosi ryzyko odrzucenia wyższych pojazdów (np. vanów).
* **Wrażliwość hiperparametrów DBSCAN:** Wartości optymalne dla jednej chmury (np. promień $\epsilon = 0.4$ m) mogą drastycznie pogorszyć wyniki na chmurach o innej gęstości skanowania (np. wzrost do $1.5$ m).
* **Współdzielony próg RANSAC:** Użycie tego samego parametru do tolerancji RANSAC i klasyfikacji gruntu powoduje, że najniższe fragmenty pojazdów (zderzaki, dolne krawędzie kół) są klasyfikowane jako droga i bezpowrotnie tracone przed etapem klastrowania. 

## Stack Technologiczny
* **Język:** C++
* **Algorytmy:** RANSAC, DBSCAN, AABB (Axis-Aligned Bounding Box), K-Nearest Neighbors (Spherical Search)
* **Zastosowanie:** Lidar Data Processing, Computer Vision, Autonomous Systems
