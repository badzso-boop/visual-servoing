# Prezentáció Vázlat - Autonóm Szerelőrobot Projekt (8-10 dia)

## 1. Dia: Cím
*   **Cím:** Vizuális Visszacsatoláson Alapuló Autonóm Szerelőrobot Platform / Moduláris Robotplatform Fejlesztése Vizuális Szervóvezérléssel
*   **Szerző:** Ujj Norbert (NLUG4F)
*   **Konzulens:** [Konzulens neve]
*   **Dátum/Esemény:** [Szakdolgozat védés, 202X. YYYY. ZZ.]

## 2. Dia: A Probléma és Motiváció
*   **A problémakör bemutatása:** Az építőipari automatizálás szükségessége (pl. munkaerőhiány, ismétlődő, precíziós feladatok).
*   **A projekt víziója:** Autonóm robotok alkalmazása épületgépészeti szerelési feladatokra (pl. konnektorok, kapcsolók).
*   **A jelenlegi kihívások:** A meglévő robotok korlátai, a feladat komplexitása.

## 3. Dia: Célkitűzések
*   **Fő cél:** Egy moduláris, részben autonóm robotplatform prototípusának fejlesztése egyszerű szerelési munkálatok elvégzésére vizuális visszacsatolás alkalmazásával.
*   **Specifikus célok:**
    *   Precíziós pozicionálás megvalósítása módosított "eye-to-hand" vizuális szervóvezérléssel.
    *   A kétfázisú kameramozgatáson alapuló Z tengely menti illesztés validálása.
    *   Egy specifikus szerelési feladat (pl. konnektor behelyezése) sikeres végrehajtásának demonstrálása.

## 4. Dia: Irodalmi Háttér és Kulcsfogalmak
*   **Vizuális szervóvezérlés (Visual Servoing):** Alapkoncepciók (IBVS - Image-Based Visual Servoing, PBVS - Position-Based Visual Servoing).
*   **Perspektív Transzformáció:** Hogyan segíti a 2D képi adatok alapján történő 3D pozicionálást (Cao C. kutatása).
*   **Képi Jellemzők Kinyerése:** A komplex (nem-geometriai) alakzatok felismerésének kihívása és a mélytanulási (pl. U-NET) megközelítések lehetőségei (Yan S. kutatása).

## 5. Dia: A Javasolt Megoldás: Rendszerarchitektúra
*   **Áttekintő blokkdiagram:** A robotplatform fő hardver- és szoftverkomponensei közötti kapcsolatok (Robotkar(ok), Kamera rendszer, Vezérlő egység/PC, Képfeldolgozó modul, Vezérlő algoritmusok).
*   **A modularitás bemutatása:** A fejlesztés fókusza a szerelőkaron és a vizuális rendszeren.

## 6. Dia: Implementáció - 1. Fázis: A Prototípus
*   **A fázis célja:** Egy működőképes, egyszerűsített demonstráció, a fő koncepciók validálása.
*   **A módosított "Eye-to-Hand" megközelítés részletei:**
    *   **Kiinduló helyzet:** Robot manuális előpozicionálása a célterület elé.
    *   **X-Y tengely menti illesztés:** Szemközti, fix kameranézetből, perspektív transzformációval a szerelék vízszintes és függőleges igazítása.
    *   **Z tengely menti illesztés (mélység):** A kamera mozgatása felülnézeti pozícióba, majd képi adatok alapján a pontos beillesztési mélység meghatározása.
    *   **Felügyelet és rögzítés:** A kamera visszaállítása az eredeti pozícióba a csavarozás vagy egyéb rögzítési műveletek monitorozásához.
*   **Miért ez a megközelítés?** (Praktikusság, időkeret, fókusz a kritikus lépésekre).

## 7. Dia: Továbbfejlesztési Lehetőségek - 2. Fázis
*   **A jövőbeli rendszer víziója:** Teljesen autonóm és dinamikus működés.
*   **A háromkaros koncepció:** Két szerelőkar és egy dedikált, harmadik, autonóm módon mozgó kamerakar.
*   **Előnyök:** Teljes autonómia, optimális nézőpont biztosítása, fokozott robusztusság és alkalmazkodóképesség.
*   **Kapcsolódás az irodalomhoz:** A Cao által leírt dinamikus IBVS rendszer teljes implementációja.

## 8. Dia: Eredmények és Demo (Várható)
*   **A prototípus demonstrációja:** Videó vagy élő bemutató a szerelési feladat sikeres végrehajtásáról.
*   **Mérési eredmények:** A pozicionálás pontosságának (pl. hibatűrés mm-ben) és a szerelési időnek az bemutatása.
*   **Konklúzió az 1. fázisról:** A módosított "eye-to-hand" megközelítés validálása.

## 9. Dia: Összefoglalás
*   **Probléma:** Az autonóm szerelés kihívása.
*   **Megoldás:** Fázisokra bontott fejlesztési terv, egy innovatív prototípus és egy jövőbeli vízió.
*   **Főbb eredmények (a prototípusból):** Egy praktikus és hatékony vizuális szervóvezérlési stratégia demonstrálása.
*   **A jövő:** A teljesen autonóm, dinamikus rendszerek felé vezető út.

## 10. Dia: Kérdések?
*   **Felirat:** Köszönöm a figyelmet!
*   **Felhívás:** Kérdések?
*   **Elérhetőségek (opcionális):** E-mail cím.
