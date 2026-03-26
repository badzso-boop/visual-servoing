# Visual Servoing Workspace – Teljes Dokumentáció

**Szerző:** Ujj Norbert (NLUG4F) – Óbudai Egyetem, NIK
**Projekt:** Vizuális visszacsatoláson alapuló robotkar vezérlőrendszer
**Verzió:** 1.0

---

> **Hogyan olvasd ezt a dokumentumot?**
> Ez a leírás feltételezi, hogy értesz a programozáshoz (Python, OOP, algoritmusok),
> de nem feltétlenül ismered a robotikát, a számítógépes látást vagy a vezérléselmélet matematikáját.
> Minden fogalmat előbb szavakkal, majd képlettel magyarázok, és mindig megmutatom,
> hol jelenik meg a kódban.

---

# Tartalomjegyzék

1. [Mi ez a projekt és mi a cél?](#1-mi-ez-a-projekt-és-mi-a-cél)
2. [A rendszer nagy képe – Hogyan működik együtt minden?](#2-a-rendszer-nagy-képe)
3. [Matematikai alapok I. – A kamera és a perspektíva](#3-matematikai-alapok-i--a-kamera-és-a-perspektíva)
4. [Matematikai alapok II. – Homográfia és a virtuális képsík](#4-matematikai-alapok-ii--homográfia-és-a-virtuális-képsík)
5. [Matematikai alapok III. – IBVS és az Interakciós Mátrix](#5-matematikai-alapok-iii--ibvs-és-az-interakciós-mátrix)
6. [Matematikai alapok IV. – ADRC és az ESO](#6-matematikai-alapok-iv--adrc-és-az-eso)
7. [Kód magyarázat – feature_detector.py](#7-kód-magyarázat--feature_detectorpy)
8. [Kód magyarázat – ibvs_controller.py](#8-kód-magyarázat--ibvs_controllerpy)
9. [Kód magyarázat – adrc_controller.py](#9-kód-magyarázat--adrc_controllerpy)
10. [Kód magyarázat – vision_node.py](#10-kód-magyarázat--vision_nodepy)
11. [Kód magyarázat – controller_node.py](#11-kód-magyarázat--controller_nodepy)
12. [Gazebo Windowson – WSL2](#12-gazebo-windowson--wsl2)
13. [Gazebo Windowson – Docker](#13-gazebo-windowson--docker)
14. [Telepítés és első indítás](#14-telepítés-és-első-indítás)
15. [Tesztelés és hibakeresés](#15-tesztelés-és-hibakeresés)
16. [Irodalom és további olvasnivalók](#16-irodalom-és-további-olvasnivalók)

---

# 1. Mi ez a projekt és mi a cél?

## Az alapkérdés

Képzeld el, hogy van egy robotkarod, és azt szeretnéd, hogy önállóan dugjon be egy csatlakozót egy fali aljzatba. Első gondolat: "Programozd be a koordinátákat!" – de ez nem működik a valóságban, mert:

- A robot nem pontosan ott áll ahol gondolod (pozicionálási hiba)
- A fali aljzat sem pontosan ott van ahol a tervrajzon szerepel
- Hőmérséklet-változás, kopás, rezgés – mind eltolják a pozíciót

A megoldás: **adj a robotnak szemet**. Ha a robot látja amit csinál, folyamatosan korrigálhatja magát – ugyanúgy ahogy te is "ránézés alapján" igazítod a kézfejed amikor valamit behelyezel.

Ez a **Visual Servoing** (vizuális szervovezérlés) lényege: a kamera képe alapján, valós időben vezéreljük a robot mozgását.

## Konkrétan mit csinál a rendszer?

```
KIINDULÁS:
  - A robot egy fal előtt áll
  - A kar végén van egy csatlakozódugasz
  - A falon van egy csatlakozóaljzat
  - Egy fix kamera látja mindkettőt

FOLYAMAT:
  1. Kamera képet kap (30 fps)
  2. Megkeresi az aljzatot és a dugaszt a képen
  3. Kiszámolja a köztük lévő eltérést (hibavektor)
  4. A hibából kiszámolja, hogy merre kell mozdítani a kart
  5. Mozgatja a kart → kisebb lesz a hiba
  6. Ismétlés amíg a hiba nulla → illesztve!
```

Ez a visszacsatolásos (feedback) rendszer alapgondolata – ugyanaz mint egy termosztátnál, csak itt képkoordináták a visszacsatolás, nem hőmérséklet.

---

# 2. A rendszer nagy képe

## Az adatfolyam

```
┌─────────────────────────────────────────────────────────────────┐
│                     FIZIKAI VILÁG                               │
│                                                                 │
│   [FALI ALJZAT]                    [KAR + DUGASZ]              │
│       (célpont)                     (mozgatandó)                │
│           │                               │                     │
│           └──────────── [KAMERA] ─────────┘                     │
│                    (mindkettőt látja)                           │
└─────────────────────────────────────────────────────────────────┘
                              │
                        képfolyam
                        (30 fps)
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      vision_node                                │
│                                                                 │
│  Kép → Előfeldolgozás → Detektálás → Homográfia → Hibavektor   │
│                                                                 │
│  Eredmény: "A dugasz 47 pixellel balra és 12 pixellel feljebb  │
│             van az aljzathoz képest, és 3cm-rel messzebb"       │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                         hibavektor
                    (error_x, error_y, error_z)
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    controller_node                               │
│                                                                 │
│  Hibavektor → ADRC → Sebességparancs                           │
│                                                                 │
│  Eredmény: "Menj 0.03 m/s sebességgel jobbra és 0.01 m/s       │
│             sebességgel le"                                      │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                      sebességparancs
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                   Robot Driver (MoveIt2 Servo)                  │
│                                                                 │
│  Cartesian sebesség → Inverz kinematika → Ízületi parancsok    │
└─────────────────────────────────────────────────────────────────┘
```

## Miért ROS2?

A **Robot Operating System 2 (ROS2)** nem egy operációs rendszer – ez egy **middleware**, azaz egy kommunikációs réteg robotikai alkalmazásokhoz.

**Analógia:** Gondolj rá úgy mint egy iroda belső levelezési rendszere. Minden részleg (node) csinál valamit, és "üzeneteket" (messages) küld egymásnak "belső postán" (topics). Nem kell hogy tudja ki olvas az üzenetből – csak beküldi, és aki kell az átveszi.

**Topic-ok** ebben a projektben:
- `/camera/image_raw` – ide kerülnek a képkockák (a kamera "beküldi", a vision_node "átveszi")
- `/vision/visual_error` – ide kerül a hibavektor (vision_node beküldi, controller_node átveszi)
- `/servo_node/delta_twist_cmds` – ide kerül a sebességparancs (controller_node beküldi, robot driver átveszi)

**Miért jó ez?** Mert a szimulációban és a valóságban ugyanezek a topic-ok vannak. Csak annyit cserélsz, hogy Gazebo helyett valódi robot driver adja az adatot – a többi kód változatlan marad.

---

# 3. Matematikai alapok I. – A kamera és a perspektíva

## A pinhole kamera modell

Mielőtt bármi mást megértenénk, meg kell értenünk hogyan "lát" egy kamera matematikailag.

**A lényeg egy mondatban:** A kamera a 3D-s valóságot levetíti egy 2D-s képsíkra, és ezt a vetítést egy mátrix írja le.

### Mi a perspektíva?

Nézz ki az ablakon. A távolabbi tárgyak kisebbnek látszanak, az egymással párhuzamos vonalak (pl. útszélek) egy pontban találkoznak a látóhatáron. Ez a **perspektív projekció**.

Matematikailag: ha van egy 3D-s pont `P = (X, Y, Z)` a kamera koordinátarendszerében (ahol Z az előre irány, a kamera optikai tengelye), akkor a képen megjelenő `p = (u, v)` koordinátái:

```
u = fx * (X/Z) + cx
v = fy * (Y/Z) + cy
```

**Mit jelent ez?**
- `X/Z` és `Y/Z`: a 3D pont "szöge" a kamerához képest
- `fx`, `fy`: fókusztávolság pixelben – ez mondja meg mennyire "nagyítja fel" a kamera a szöget
- `cx`, `cy`: a kép közepének koordinátái (főpont) – mert a kamera nem feltétlenül pontosan középre képez

### A K mátrix (Intrinsic Matrix)

Ezt az összefüggést mátrixos formában írjuk fel. Homogén koordinátákat használva (ahol minden vektort egy extra 1-essel bővítünk) a vetítés:

```
    ⎡u⎤   ⎡fx   0   cx⎤   ⎡X/Z⎤
s * ⎢v⎥ = ⎢ 0   fy  cy⎥ * ⎢Y/Z⎥
    ⎣1⎦   ⎣ 0   0    1⎦   ⎣ 1 ⎦
```

ahol `s` egy skálázási tényező (ami éppen Z lesz).

A középső 3×3-as mátrix a **K mátrix** (intrinsic matrix):

```
    ⎡fx   0   cx⎤
K = ⎢ 0   fy  cy⎥
    ⎣ 0   0    1⎦
```

**Ezt a kalibrálás határozza meg** – az a folyamat ahol sakktáblás mintával "betanítjuk" a kamerát, hogy megtudjuk a pontos értékeket.

### Miért van fx és fy külön?

Mert a pixelek nem feltétlenül négyzet alakúak, és a gyártási hibák is számítanak. A legtöbb modern kameránál `fx ≈ fy`, de a pontos értékek kalibráció nélkül nem ismertek.

### Kódban ez hol jelenik meg?

A `vision_node.py`-ban:
```python
def camera_info_callback(self, msg: CameraInfo):
    K = msg.k   # 3×3 intrinsic mátrix, sorfolytonosan
    self.fx = K[0]   # K[0,0]
    self.fy = K[4]   # K[1,1]
    self.cx = K[2]   # K[0,2]
    self.cy = K[5]   # K[1,2]
```

Az `ibvs_controller.py`-ban az interakciós mátrix számításánál:
```python
x = (u - self.cx) / self.fx   # normalizált koordináta
y = (v - self.cy) / self.fy
```

---

# 4. Matematikai alapok II. – Homográfia és a virtuális képsík

## Mi a homográfia?

**Egyszerű magyarázat:** Ha két kamera ugyanazt a sík felületet fényképezi különböző szögekből, a két kép közötti kapcsolatot egy 3×3-as mátrix írja le. Ezt hívják homográfiának.

**Konkrét példa a projektből:** Az aljzat egy sík felület (a fal). Ha a kamera kissé oldalirányból néz, az aljzat torzítottan látszik (trapéz alakú). A homográfia mátrix segítségével ezt a torzított képet "kicsinálhatjuk" – mintha egyenesen szemből néznénk.

```
AMIT A KAMERA LÁT          AMIT A HOMOGRÁFIA AD
(oldalirányból):            (szemből, "rectified"):

  ╱──────╲                    ┌──────┐
 ╱        ╲        H →        │      │
╱  aljzat  ╲                  │aljzat│
╲          ╱                  │      │
 ╲________╱                   └──────┘
```

## Matematikai levezetés

Két kép (`I₁` és `I₂`) homográfiával kapcsolódik egymáshoz ha:

```
p₂ = H * p₁
```

ahol `p₁ = [u₁, v₁, 1]ᵀ` és `p₂ = [u₂, v₂, 1]ᵀ` homogén képkoordináták, és `H` egy 3×3-as mátrix.

Kifejtve:
```
    ⎡h₁  h₂  h₃⎤   ⎡u₁⎤   ⎡u₂'⎤
s * ⎢h₄  h₅  h₆⎥ * ⎢v₁⎥ = ⎢v₂'⎥
    ⎣h₇  h₈  h₉⎦   ⎣ 1⎦   ⎣ s ⎦
```

majd `u₂ = u₂'/s` és `v₂ = v₂'/s`.

A H mátrixnak **9 eleme van, de csak 8 fokú szabadsága** (mert skáláig meghatározott) – ezért legalább **4 pontpár** kell a meghatározásához (minden pont 2 egyenletet ad → 4×2=8 egyenlet = 8 ismeretlen).

## A DLT módszer (Direct Linear Transformation)

Ha ismerjük a megfelelő pontpárokat `(p₁ᵢ ↔ p₂ᵢ)`, a H mátrix meghatározható lineáris algebra segítségével.

**Az egyenlet:** `p₂ = H * p₁` átírható `A * h = 0` formájúra, ahol `h` a H mátrix elemeinek vektora és `A` az adatmátrix.

Minden pontpárból 2 sor kerül az A mátrixba:
```
sor₁: [-u₁, -v₁, -1,  0,   0,   0,  u₂*u₁, u₂*v₁, u₂]
sor₂: [ 0,   0,   0, -u₁, -v₁, -1,  v₂*u₁, v₂*v₁, v₂]
```

Az `A * h = 0` egyenlet megoldása az **SVD (Singular Value Decomposition)** segítségével: a megoldás az A mátrix legkisebb szinguláris értékéhez tartozó jobbszinguláris vektor.

```python
# Ez történik az OpenCV findHomography() belsejében:
U, S, Vt = np.linalg.svd(A)
h = Vt[-1]          # utolsó sor (legkisebb szinguláris érték)
H = h.reshape(3, 3)
H = H / H[2, 2]    # normalizálás
```

**A RANSAC robusztusítás:** A valóságban a pontpárok között lehetnek hibások (outlier-ek). A RANSAC (Random Sample Consensus) véletlenszerűen választ 4 pontpárt, kiszámolja a H-t, majd megnézi hány pont egyezik ezzel a H-val. A legtöbb egyezőt adó H nyeri.

```python
# ibvs_controller.py-ban:
H, mask = cv2.findHomography(
    src_points.astype(np.float32),
    dst_points.astype(np.float32),
    cv2.RANSAC,    # robusztus becslés
    5.0            # maximális pixelhiba ami még "inlier"-nek számít
)
```

## A virtuális képsík – Cao (2022) kulcsötlete

A hagyományos IBVS probléma: ha az aljzat és a dugasz képi koordinátáit közvetlenül hasonlítjuk össze, az oldalirányos kameranézőpont torzítást visz a hibavektorba.

**Cao megoldása:** Hozzunk létre egy "virtuális képsíkot" (`γ̃`) ami párhuzamos a fallal. Vetítsük mindkét objektumot erre a síkra. Ezen a síkon az összehasonlítás torzításmentes.

```
KAMERA KÉPE                    VIRTUÁLIS SÍK (γ̃)
(torzított nézőpont)           (fal síkjával párhuzamos)

 ╭───────╮                      ┌─────────┐
 │aljzat │  ──── H_a ────►      │  ˜f_α  │
 ╰───────╯                      │    ↕   │  ← hiba itt 0 ha illesztve
 ╭───────╮  ──── H_b ────►      │  ˜f_β  │
 │ dugasz│                      └─────────┘
 ╰───────╯
```

Az illesztési feltétel: `f̃_α = f̃_β` (mindkét objektum ugyanoda vetül a virtuális síkon)

A hibavektor: `e = f̃_α - f̃_β`

Ha `e = 0`, a dugasz pontosan az aljzat előtt van (csak Z irányban különböznek).

---

# 5. Matematikai alapok III. – IBVS és az Interakciós Mátrix

## Mi az IBVS?

**Image-Based Visual Servoing** – a robot mozgását kizárólag képi koordináták alapján vezéreljük, nem szükséges a 3D-s rekonstrukció.

**Analógia:** Képzeld el, hogy becsukott szemmel kell céltáblát dobni. Ezt a robot koordinátákkal csinálná. Az IBVS olyan, mintha nyitott szemmel dobnál – a szemed képe alapján korrigálod a kézfejed mozgását, anélkül hogy tudnád pontosan hány centiméter a távolság.

## A hibavektor

Legyenek `s` az aktuális képi jellemzők (pl. az aljzat 4 sarokpontjának koordinátái), és `s*` a kívánt értékek (amikor illesztve van). A hibavektor:

```
e(t) = s(t) - s*
```

A cél: `e → 0` amint `t → ∞`

A mi esetünkben `s` a dugasz középpontja a virtuális képsíkon, `s*` az aljzat középpontja – és azt akarjuk hogy egybeessen.

## Az Interakciós Mátrix (Image Jacobian)

**A kulcskérdés:** Ha a kamera (és az end-effector) egy `v_c = [vx, vy, vz, ωx, ωy, ωz]` sebességgel mozog, mennyire változnak a képi jellemzők?

Ezt a kapcsolatot írja le az **interakciós mátrix L**:

```
ṡ = L * v_c
```

ahol `ṡ` a képi jellemzők időbeli deriváltja, `v_c` a kamera sebességvektora.

### Levezetés egy képi pontra

Legyen `p = (u, v)` egy képi pont, amelyik a 3D-s `P = (X, Y, Z)` ponthoz tartozik. A pinhole modell szerint:

```
u = fx * X/Z + cx     →    X/Z = (u - cx)/fx = x (normalizált)
v = fy * Y/Z + cy     →    Y/Z = (v - cy)/fy = y (normalizált)
```

**1. lépés:** Írjuk fel `X, Y, Z` időbeli deriváltját ha a kamera `v_c = [vx, vy, vz, ωx, ωy, ωz]` sebességgel mozog.

A 3D pont mozgása a kamera koordinátarendszerében (a 3D pont fix a világban, a kamera mozog):
```
Ẋ = -vx + ωy*Z - ωz*Y
Ẏ = -vy + ωz*X - ωx*Z
Ż = -vz + ωx*Y - ωy*X
```

**2. lépés:** Számítsuk ki `u̇` és `v̇` értékét a lánc-szabály alkalmazásával:

```
u̇ = fx * d(X/Z)/dt = fx * (Ẋ*Z - X*Ż) / Z²
```

Behelyettesítve az előző összefüggéseket és rendezve:

```
u̇ = -fx/Z * vx  +  0 * vy  +  (u-cx)/Z * vz
   + (u-cx)(v-cy)/fy * ωx
   - (fx² + (u-cx)²/fx) * ωy
   + (v-cy) * ωz
```

Normalizált koordinátákban (`x = (u-cx)/fx`, `y = (v-cy)/fy`):

```
u̇ = -fx/Z * vx  +  x/Z * vz  +  x*y * ωx  -  (1+x²) * ωy  +  y * ωz
v̇ = -fy/Z * vy  +  y/Z * vz  +  (1+y²) * ωx  -  x*y * ωy  -  x * ωz
```

**3. lépés:** Mátrix formában, egy pontra:

```
        ⎡u̇⎤   ⎡-fx/Z    0    x/Z    x*y    -(1+x²)    y ⎤   ⎡vx⎤
        ⎢  ⎥ = ⎢                                          ⎥ * ⎢vy⎥
        ⎣v̇⎦   ⎣  0    -fy/Z  y/Z  (1+y²)    -x*y     -x ⎦   ⎢vz⎥
                                                               ⎢ωx⎥
                                                               ⎢ωy⎥
                                                               ⎣ωz⎦
```

Ez a 2×6-os mátrix egy pont **részleges interakciós mátrixa**. 4 sarokpont esetén 4 darab ilyen mátrixot egymás alá rakva kapjuk a teljes 8×6-os L mátrixot.

### Kódban (ibvs_controller.py):

```python
def compute_interaction_matrix(self, points, Z=None):
    Z = Z or self.Z_estimate
    N = len(points)
    L = np.zeros((2 * N, 6))

    for i, (u, v) in enumerate(points):
        # Normalizált koordináták
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy

        row = 2 * i
        # x irányú sor
        L[row, 0] = -1.0 / Z          # vx hatása u̇-ra
        L[row, 1] = 0.0
        L[row, 2] = x / Z             # vz hatása u̇-ra
        L[row, 3] = x * y             # ωx hatása u̇-ra
        L[row, 4] = -(1.0 + x ** 2)  # ωy hatása u̇-ra
        L[row, 5] = y                 # ωz hatása u̇-ra

        # y irányú sor (hasonlóan)
        L[row+1, 1] = -1.0 / Z
        ...
```

## A vezérlési törvény levezetése

**Célunk:** `e(t) → 0` exponenciálisan. Ez azt jelenti:

```
ė = -λ * e
```

ahol `λ > 0` egy vezérlési erősítés. Ez a feltétel garantálja az exponenciális konvergenciát: ha `e(0)` a kezdeti hiba, akkor `e(t) = e(0) * e^(-λt)`.

**Mivel** `ṡ = L * v_c` és `e = s - s*`, ezért `ė = ṡ - ṡ* = ṡ` (mert `s*` konstans):

```
ė = L * v_c = -λ * e
```

Ebből a szükséges sebességparancs:

```
v_c = -λ * L⁺ * e
```

ahol `L⁺` az L mátrix pszeudo-inverze (mert L nem négyzetes, nem invertálható közvetlenül).

### A pszeudo-inverz

Az `L⁺` (Moore-Penrose pszeudo-inverz) minimális hosszú megoldást ad:

```
L⁺ = (Lᵀ * L)⁻¹ * Lᵀ
```

A `numpy.linalg.pinv()` ezt SVD segítségével számolja, ami numerikusan stabil:

```python
# ibvs_controller.py:
L_pinv = np.linalg.pinv(L)
velocity = -self.lam * L_pinv @ error
```

### Miért converges ez?

Ha behelyettesítjük az ideális esetbe (ahol `L` pontosan ismert):

```
ė = L * v_c = L * (-λ * L⁺ * e) = -λ * (L * L⁺) * e
```

Ha L teljes rangú (ami 4 pontpárral teljesül), akkor `L * L⁺ = I` (egységmátrix), tehát:

```
ė = -λ * e    →    e(t) = e(0) * e^(-λt)
```

Ez exponenciális konvergencia. A `λ` erősítés szabályozza a sebességet:
- Nagy `λ` → gyors, de zajra érzékeny, lehet instabil
- Kis `λ` → lassú, de stabil

---

# 6. Matematikai alapok IV. – ADRC és az ESO

## Miért nem elég az IBVS önmagában?

Az IBVS feltételezi, hogy az L mátrix pontosan ismert. A valóságban azonban:

1. **A mélység (Z) pontatlan** – a becslési hiba eltéríti a vezérlőjelet
2. **A robot dinamikája nem ideális** – súrlódás, rugalmasság, inercia
3. **Rezgések, külső erők** – a valódi környezetben mindig van zaj

Ezek mind "zavarként" (disturbance) jelennek meg a rendszerben, és az IBVS nem kompenzálja őket.

## Az ADRC alapgondolata

**Han Jingqing** (1998) felismerése: ahelyett hogy pontosan modelleznénk a rendszert, **becsüljük meg és kompenzáljuk a zavarokat valós időben**.

**Analógia:** Képzeld el, hogy egy hajót vezetsz és szél fúj. Két lehetőség:
1. Pontos meteorológiai modell alapján kompenzálod előre a szelet (hagyományos vezérlés)
2. Nézed a hajó eltérését, és érzed a szél hatását, majd kompenzálod amit éppen látsz (ADRC)

A második megközelítés robusztusabb, mert nem kell precíz modell.

## A rendszermodell

Az ADRC legegyszerűbb formájában egy másodfokú rendszert feltételez:

```
ÿ = f(y, ẏ, d, t) + b₀ * u
```

ahol:
- `y` – a kimenet (pl. képi koordináta)
- `u` – a vezérlőjel (pl. sebességparancs)
- `b₀` – a bemenet erősítésének közelítő értéke (ezt meg kell becsülnünk)
- `f(y, ẏ, d, t)` – az összes zavar összessége (modellezési hibák, külső erők, nemlinearitások)

Az ADRC-ben az `f` függvényt **kiterjesztett állapotként** kezeljük és az ESO becsüli.

## A Lineáris ESO (Extended State Observer)

Vezessünk be három állapotváltozót:

```
x₁ = y          (kimenet)
x₂ = ẏ          (kimenet deriváltja / sebesség)
x₃ = f(...)     (teljes zavar – ez az "extended" állapot)
```

Az állapotegyenletek:

```
ẋ₁ = x₂
ẋ₂ = x₃ + b₀ * u
ẋ₃ = ḟ ≈ 0      (lassú zavar változást feltételezünk)
```

Az ESO egy Luenberger-megfigyelő erre a rendszerre:

```
ż₁ = z₂ - β₁ * (z₁ - y)
ż₂ = z₃ - β₂ * (z₁ - y) + b₀ * u
ż₃ =    - β₃ * (z₁ - y)
```

ahol `(z₁ - y)` a megfigyelési hiba (becsült kimenet mínusz mért kimenet), és `β₁, β₂, β₃` az observer erősítések.

### Az erősítések meghatározása sávszélességből

Az erősítések úgy választhatók, hogy az observer karakterisztikus polinomja `(s + ω_o)³` legyen:

```
β₁ = 3 * ω_o
β₂ = 3 * ω_o²
β₃ = ω_o³
```

ahol `ω_o` az observer sávszélessége (rad/s). **Egyetlen paramétert kell hangolni**, ami nagy előny!

**Általános szabály:** `ω_o = 3-10 × zárt hurok sávszélesség (ω_c)`

### Kódban (adrc_controller.py):

```python
class LinearESO:
    def __init__(self, bandwidth, b0, dt):
        w = bandwidth
        self.beta1 = 3 * w        # 3ω_o
        self.beta2 = 3 * w ** 2   # 3ω_o²
        self.beta3 = w ** 3       # ω_o³

    def update(self, y, u):
        e_obs = self.z1 - y   # megfigyelési hiba

        # Euler integrálás (diszkrét lépés)
        dz1 = self.z2 - self.beta1 * e_obs
        dz2 = self.z3 - self.beta2 * e_obs + self.b0 * u
        dz3 =         - self.beta3 * e_obs

        self.z1 += self.dt * dz1
        self.z2 += self.dt * dz2
        self.z3 += self.dt * dz3
```

## A zavar kompenzáció

Ha az ESO jól becsüli `z₃ ≈ f`, akkor kompenzálhatjuk a zavart:

```
u = (u₀ - z₃) / b₀
```

ahol `u₀` egy egyszerű PD vezérlő kimenete:

```
u₀ = kp * (r - z₁) + kd * (ṙ - z₂)
```

Behelyettesítve a rendszerbe:

```
ÿ = f + b₀ * u = f + b₀ * (u₀ - z₃)/b₀ = f + u₀ - z₃ ≈ u₀
```

(mert `z₃ ≈ f`). Tehát a zavar-kompenzált rendszer egyszerű kettős integrátorként viselkedik, amelyet a PD szabályozó könnyen vezérelhet!

### Kódban:

```python
def compute(self, reference, measurement, reference_dot=0.0):
    z1, z2, z3 = self.eso.update(measurement, self._last_u)

    e1 = reference - z1           # pozíció hiba
    e2 = reference_dot - z2       # sebesség hiba

    u0 = self.kp * e1 + self.kd * e2   # PD rész

    u = (u0 - z3) / self.b0            # zavar kompenzáció

    self._last_u = u
    return u
```

---

# 7. Kód magyarázat – feature_detector.py

## Mi a fájl célja?

Ez a fájl teljesen független a ROS2-től – csak OpenCV és NumPy kell hozzá. **Feladata:** a kamera képén megtalálni az aljzatot és a dugaszt, és visszaadni azok képi koordinátáit.

## ConnectorDetector – Az aljzat megkeresése

### Miért Canny éldetektálás?

Az aljzat (fehér műanyag kerettel) jól látható kontrasztot képez a fallal. Az élek (ahol a szín hirtelen változik) pontosan jelölik ki a keret határait.

**A Canny algoritmus lépései:**
1. Gaussian szűrés (zaj eltávolítása)
2. Gradiens számítás (Sobel operátor)
3. Non-maximum suppression (csak az erős élek maradnak)
4. Hysteresis küszöbölés (alsó és felső küszöb)

```python
edges = cv2.Canny(processed, self.canny_low, self.canny_high)
```

**A két küszöb értelmezése:**
- Pixel erőssége > `canny_high` → biztosan él
- Pixel erőssége < `canny_low` → biztosan nem él
- Közte → csak akkor él, ha egy "biztosan él" pixelhez kapcsolódik

### Miért CLAHE az előfeldolgozásnál?

```python
self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
enhanced = self.clahe.apply(blurred)
```

**CLAHE = Contrast Limited Adaptive Histogram Equalization**

Az épületek falain a megvilágítás egyenetlen – egy ablak mellől jövő fény árnyékot vet, a lámpa pedig egy területet felülmegvilágít. A normál hisztogram-egyenlítés ezt az egész képre egyszerre csinálja, ami torzíthat. A CLAHE a képet kis blokkokra osztja (`tileGridSize=(8,8)` = 8×8 blokkra), és minden blokkot külön egyenlít ki. A `clipLimit` megakadályozza a túlzott felerősítést.

**Referencia:** [OpenCV CLAHE dokumentáció](https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html)

### A kontúr szűrési logika

```python
def _find_best_contour(self, contours):
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (self.min_area < area < self.max_area):
            continue

        _, (w, h), _ = cv2.minAreaRect(cnt)
        ratio = w / h
        if not (self.min_ratio < ratio < self.max_ratio):
            continue

        hull = cv2.convexHull(cnt)
        solidity = area / cv2.contourArea(hull)
        if solidity < 0.7:
            continue
```

**Miért háromféle szűrést alkalmazunk?**

1. **Terület szűrés:** A csatlakozó ismert fizikai méretű (~80×80mm). A kamera távolságától függően ennek képi mérete becsülhető. Kisebb tárgyak zajok, nagyobbak a fal maga.

2. **Arány szűrés:** A csatlakozó közel négyzet alakú (arány 0.5–2.0 között). Hosszú vékony kontúrok (pl. repedések a falon) kiesnek.

3. **Konvexitás (solidity):** `solidity = terület / konvex burok területe`. Ha egy kontúr nagyon "csipkés" (pl. növény, dísz), a solidity alacsony. Az aljzat sima, egyszerű alakú → solidity > 0.7.

### Sarokpontok rendezése

```python
def _sort_corners(self, corners):
    s = corners.sum(axis=1)
    d = np.diff(corners, axis=1).flatten()

    sorted_corners[0] = corners[np.argmin(s)]  # bal-felső
    sorted_corners[2] = corners[np.argmax(s)]  # jobb-alsó
    sorted_corners[1] = corners[np.argmin(d)]  # jobb-felső
    sorted_corners[3] = corners[np.argmax(d)]  # bal-alsó
```

**Miért kell rendezni?** A homográfia számításánál a pontpároknak egyezniük kell – az aljzat bal-felső sarkát a dugasz bal-felső sarkával kell párosítani. Ha a sarokpontok random sorrendben jönnek, a homográfia hibás lesz.

**A trükk:** A bal-felső saroknak a legkisebb `u + v` összege van (legközelebb az origóhoz = bal-felső). A jobb-alsónak a legnagyobb. A másik kettőt az `u - v` különbség különbözteti meg.

## PlugDetector – A dugasz megkeresése

Két módszer:

**Szín alapú:** Ha a dugaszon kék jelölő van, a HSV-ben könnyű szűrni:
```python
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
```
**Miért HSV és nem RGB?** Az RGB szín érzékeny a megvilágítás erősségére (árnyékban sötétebb ugyanaz a szín). A HSV-ben a `H` (hue = árnyalat) szétválasztja a szín-tulajdonságot a fényerőtől → robusztusabb.

**Kör alapú (Hough):** Ha a dugasz vége kör alakú, a Hough transzformáció megtalálja:
```python
circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, ...)
```
A Hough transzformáció minden képi pontból "szavaz" arra, hogy melyik kör közepéhez tartozhat. A legtöbb szavazatot kapó (cx, cy, r) hármas a megtalált kör.

---

# 8. Kód magyarázat – ibvs_controller.py

## HomographyManager

### calibrate_from_reference()

```python
def calibrate_from_reference(self, socket_ref_points, plug_ref_points):
    self.H_a, _ = cv2.findHomography(
        socket_ref_points.astype(np.float32),
        socket_ref_points.astype(np.float32)
    )
    self.H_b, _ = cv2.findHomography(
        plug_ref_points.astype(np.float32),
        socket_ref_points.astype(np.float32)
    )
```

**Mit csinál ez?**

Ez egy egyszerűsített kalibráció. A "tanítási" fázisban manuálisan beillesztjük a dugaszt az aljzatba, és lefotózzuk a jellemzőpontokat. Ebből:

- `H_a`: az aljzat pontjait önmagára képezi le (egységmátrix közel) → a virtuális sík maga az aljzat referencia képsík
- `H_b`: a dugasz pontjait az aljzat pontjaira képezi le → kalibráláskor a dugasz pontosan az aljzat felett volt

Normál működéskor: ha a dugasz elmozdult az aljzattól, a `H_b` már nem fogja pontosan egymásra vetíteni őket → ez a hiba!

### project_to_virtual() és compute_error()

```python
def project_to_virtual(self, points, H):
    pts = points.reshape(-1, 1, 2).astype(np.float32)
    projected = cv2.perspectiveTransform(pts, H)
    return projected.reshape(-1, 2)

def compute_error(self, socket_points, plug_points):
    socket_virtual = self.project_to_virtual(socket_points, self.H_a)
    plug_virtual   = self.project_to_virtual(plug_points,   self.H_b)
    error = socket_virtual - plug_virtual
    return error.flatten()
```

**A `perspectiveTransform`** az OpenCV függvénye, ami a 3. fejezetben leírt `p' = H * p` műveletet végzi homogén koordinátákban, vektorizáltan minden pontra.

**A reshape(-1, 1, 2):** Az OpenCV perspectiveTransform-ja `(N, 1, 2)` alakú tömböt vár – ez az "N db 1-pixel-wide kontúr" konvenció az OpenCV-ben.

## IBVSController

### A legfontosabb: compute_velocity()

```python
def compute_velocity(self, error, feature_points):
    error_norm = np.linalg.norm(error)
    converged = error_norm < self.convergence_threshold

    if converged:
        return IBVSResult(velocity=np.zeros(6), ...)

    L = self.compute_interaction_matrix(feature_points, self.Z_estimate)
    L_pinv = np.linalg.pinv(L)
    velocity = -self.lam * L_pinv @ error
    velocity = self._clamp_velocity(velocity)
```

**Lépések:**
1. `np.linalg.norm(error)` – az euklideszi norma: `√(e₁² + e₂² + ... + eₙ²)`. Ez a hiba "nagysága".
2. Ha a hiba kisebb mint a küszöb → konvergált, nulla sebességet adunk vissza.
3. `compute_interaction_matrix()` – a matematikai levezetésből (5. fejezet) implementálva.
4. `np.linalg.pinv(L)` – SVD-alapú pszeudo-inverz. [NumPy dokumentáció](https://numpy.org/doc/stable/reference/generated/numpy.linalg.pinv.html)
5. `@ ` – NumPy mátrix-vektor szorzás operátor (Python 3.5+)
6. `_clamp_velocity()` – biztonsági korlátok (lásd lent)

### Biztonsági sebesség korlátozás

```python
def _clamp_velocity(self, velocity):
    linear = clamped[:3]
    linear_norm = np.linalg.norm(linear)
    if linear_norm > self.max_linear_vel:
        clamped[:3] = linear * (self.max_linear_vel / linear_norm)
```

**Miért így és nem egyszerűen `np.clip()`?** Az `np.clip()` minden tengelyt külön vágna le, ami megváltoztatná a mozgás irányát. Ez a módszer megtartja az irányt, csak a sebességet csökkenti – mint ahogy egy autónál a sebességkorlátozó sem fordul el, csak lassít.

## DepthEstimator

```python
def estimate(self, apparent_size_px):
    Z = (self.f * self.D_real) / apparent_size_px
    return float(np.clip(Z, 0.05, 2.0))
```

Ez a **hasonló háromszögek** elvéből következik:

```
f / Z = D_képen / D_valódi
```

ahol `f` a fókusztávolság, `Z` a mélység, `D_valódi` az aljzat tényleges mérete, `D_képen` a képen mért mérete. Átrendezve: `Z = f * D_valódi / D_képen`.

---

# 9. Kód magyarázat – adrc_controller.py

## LinearESO – A Kiterjesztett Állapotmegfigyelő

```python
class LinearESO:
    def __init__(self, bandwidth, b0, dt):
        w = bandwidth
        self.beta1 = 3 * w
        self.beta2 = 3 * w ** 2
        self.beta3 = w ** 3
```

Az erősítések meghatározása a 6. fejezetben levezetett `(s + ω_o)³` karakterisztikus polinom alapján. Ez biztosítja, hogy az observer stabilan és elég gyorsan konvergál.

### Az Euler integrálás

```python
def update(self, y, u):
    e_obs = self.z1 - y

    dz1 = self.z2 - self.beta1 * e_obs
    dz2 = self.z3 - self.beta2 * e_obs + self.b0 * u
    dz3 =         - self.beta3 * e_obs

    self.z1 += self.dt * dz1
    self.z2 += self.dt * dz2
    self.z3 += self.dt * dz3
```

**Miért Euler integrálás?** Az Euler módszer (`x(t+dt) = x(t) + dt * ẋ(t)`) a legegyszerűbb numerikus integrálási módszer. A mi esetünkben `dt = 0.033s` (30 Hz), ami elég kicsi a mi dinamikánkhoz, így az Euler módszer elfogadhatóan pontos.

**Pontosabb alternatíva:** Runge-Kutta 4 (RK4), de ennél az alkalmazásnál nem szükséges.

### Miért három állapot?

- `z1 ≈ y`: a kimenet becslése. Az `e_obs = z1 - y` visszacsatolás korrigálja, ha eltér a mérttől.
- `z2 ≈ ẏ`: a sebesség becslése – nem mérjük közvetlenül, az ESO következteti ki a kimenet változásából.
- `z3 ≈ f`: a teljes zavar becslése. Ez az ADRC "varázslata" – anélkül becsüli a zavart, hogy tudná mi okozza.

## ADRCController

```python
def compute(self, reference, measurement, reference_dot=0.0):
    z1, z2, z3 = self.eso.update(measurement, self._last_u)

    e1 = reference - z1
    e2 = reference_dot - z2

    u0 = self.kp * e1 + self.kd * e2
    u = (u0 - z3) / self.b0

    self._last_u = u
    return u
```

**Miért kell `self._last_u`?** Az ESO-nak szüksége van az előző lépésben alkalmazott vezérlőjelre (`u`), mert a rendszer dinamikájába bele van számolva a `b0 * u` tag. Ez egy kauzális (ok-okozati) visszacsatolás.

**A PD erősítések:**
```python
wc = control_bandwidth
self.kp = wc ** 2   # ω_c²
self.kd = 2 * wc    # 2ω_c
```

Ezek az erősítések a kritikusan csillapított másodrendű rendszer feltételéből jönnek: a zárt hurkú pólus `(s + ω_c)²`, amiből `kp = ω_c²` és `kd = 2ω_c`.

## MultiAxisADRC

```python
class MultiAxisADRC:
    def __init__(self, config):
        self.controllers = {
            "x": ADRCController(...),
            "y": ADRCController(...),
            "z": ADRCController(...),
        }

    def compute(self, error_x, error_y, error_z):
        vx = self.controllers["x"].compute(reference=0.0, measurement=-error_x)
        vy = self.controllers["y"].compute(reference=0.0, measurement=-error_y)
        vz = self.controllers["z"].compute(reference=0.0, measurement=-error_z)
        return np.array([vx, vy, vz])
```

**Miért `measurement=-error_x`?** A vezérlőnek egy referencia értéket és egy mérési értéket adunk. Mi azt akarjuk, hogy a hiba legyen nulla, tehát `reference=0`. A "mérési értékünk" maga az `error`, de negálva, mert: ha a hiba pozitív (dugasz jobbra van az aljzattól), a vezérlő negatív (balra mozdítsa) sebességet adjon.

---

# 10. Kód magyarázat – vision_node.py

## A ROS2 Node felépítése

```python
class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        # ...
        self.create_subscription(Image, "/camera/image_raw",
                                  self.image_callback, 10)
        self.error_pub = self.create_publisher(
            Twist, "/vision/visual_error", 10)
```

**A `10` szám a QoS queue size** – maximum 10 üzenet várakozhat feldolgozásra. Ha a vision_node lassabb mint a kamera (pl. nehéz jelenet), az régi üzenetek eldobódnak, és csak a legfrissebb 10 marad.

**Miért `Twist` üzenettípus a hibavektornak?** A `Twist` egy ROS2 standard üzenet `[linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]` komponensekkel. Mi ezt "újrahasználjuk" az `[error_x, error_y, error_z, 0, 0, 0]` tárolására. Ez nem a legszebb design, de egyszerű és nem igényel egyedi üzenettípust.

## Az image_callback logikája

```python
def image_callback(self, msg: Image):
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    socket_result = self.socket_detector.detect(image)
    plug_result   = self.plug_detector.detect(image)

    detected = socket_result.success and plug_result.success
    self.detected_pub.publish(Bool(data=detected))

    if not detected:
        self._publish_zero_error()
        return
```

**A `cv_bridge`:** A ROS2 `sensor_msgs/Image` üzenet és az OpenCV `numpy.ndarray` különböző formátumú – a `CvBridge` végzi az átalakítást. A `bgr8` kódolás az OpenCV alapértelmezett sorrendje (Blue-Green-Red), szemben az RGB-vel.

**Miért publikáljuk a "target_detected" flaget külön?** Mert a controller_node-nak tudnia kell, hogy nincs-e detektálás – ilyenkor WAITING állapotba kell lépnie, nem küldhet sebességparancsot.

## A kalibrálás nélküli mód

```python
if self.calibrated:
    error_vec = self.homography_mgr.compute_error(...)
    error_x = float(error_xy[:, 0].mean())
    error_y = float(error_xy[:, 1].mean())
else:
    error_x = float(socket_result.center[0] - plug_result.center[0])
    error_y = float(socket_result.center[1] - plug_result.center[1])
    self.get_logger().warn("Nincs homográfia kalibráció...", throttle_duration_sec=5.0)
```

**`throttle_duration_sec=5.0`:** Ez a ROS2 logging throttle – a figyelmeztetés csak 5 másodpercenként egyszer jelenik meg, hogy ne árassza el a terminált ugyanazzal az üzenettel.

---

# 11. Kód magyarázat – controller_node.py

## Az állapotgép (State Machine)

```python
class AssemblyState(Enum):
    WAITING      = auto()
    XY_ALIGNMENT = auto()
    Z_APPROACH   = auto()
    DOCKING      = auto()
    DONE         = auto()
    ERROR        = auto()
```

**Miért állapotgép?** Az illesztési feladat sorrendezési problémát jelent: csak akkor közelíthetünk Z irányban, ha az XY igazítás kész. Az állapotgép ezt strukturáltan kezeli, és minden állapotban más a vezérlési logika.

## A vezérlő hurok időzítése

```python
dt = 1.0 / rate_hz
self.timer = self.create_timer(dt, self.control_loop)
```

**Miért timer és nem a subscription callback-ben?** A kamera 30 Hz-en küld képet, de a vezérlőnek is 30 Hz-en kell futnia, függetlenül attól, hogy érkezett-e új kép. Ha a vezérlő csak képkocka-érkezéskor futna, egy elveszett kép megállítaná a robotot.

## Az XY igazítás fázis

```python
elif self.state == AssemblyState.XY_ALIGNMENT:
    adrc_out = self.adrc.compute(ex, ey, 0.0)
    velocity[0] = adrc_out[0]   # vx
    velocity[1] = adrc_out[1]   # vy

    if abs(ex) < self.xy_th and abs(ey) < self.xy_th:
        self._transition(AssemblyState.Z_APPROACH)
```

**Miért nem az IBVS-t használjuk itt is?** Valójában az ADRC és az IBVS komplementer módszerek. Az IBVS a kamera geometriájából számítja a mozgásirányt (L mátrix alapján), az ADRC pedig a zavarokat kompenzálja. A teljesen helyes implementációban az IBVS adja az alapsebesség irányt, az ADRC pedig módosítja a zavarok alapján. Ebben az egyszerűsített verzióban az ADRC önmagában vezérli az XY mozgást, az IBVS a pontos L mátrix számításra koncentrál.

## A dokkolási feltétel

```python
elif self.state == AssemblyState.DOCKING:
    velocity[2] = self.dock_spd

    if ez < self.z_th * 0.3:
        self.consecutive_done += 1
        if self.consecutive_done >= 5:
            self._transition(AssemblyState.DONE)
    else:
        self.consecutive_done = 0
```

**Miért `consecutive_done >= 5`?** Egyetlen képkockán alapuló döntés zajra érzékeny lehet. Ha 5 egymást követő képkockán (≈166ms) teljesül a feltétel, biztosan nem véletlen zaj okozta.

## A sebességparancs küldése

```python
def _publish_velocity(self, velocity):
    msg = TwistStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = "tool0"
    msg.twist.linear.x  = velocity[0]
    ...
    self.vel_pub.publish(msg)
```

**`TwistStamped` vs `Twist`:** A `TwistStamped` tartalmaz egy `header`-t timestamp-pel és frame_id-vel. A MoveIt2 Servo elvárja ezt, mert tudnia kell *mikor* keletkezett a parancs és *melyik koordinátarendszerben* értendő. A `tool0` a UR robot végpontjának (end-effector) koordinátarendszere.

---

# 12. Gazebo Windowson – WSL2

## Mi a WSL2?

A **Windows Subsystem for Linux 2** egy Microsoft által fejlesztett technológia, amely lehetővé teszi, hogy teljes értékű Linux kernelt futtass Windows 10/11-en, virtuális géphez hasonlóan, de sokkal kisebb erőforrásigénnyel és jobb integrációval.

**Röviden:** Ubuntu fut a Windowsodban, és a Gazebo + ROS2 ott fut. A grafikus ablak (Gazebo 3D nézet) a Windows asztalon jelenik meg.

## Telepítési lépések

### 1. lépés – WSL2 és Ubuntu telepítése

Nyiss egy **PowerShell**-t rendszergazdaként és futtasd:

```powershell
wsl --install
```

Ez automatikusan telepíti a WSL2-t és az Ubuntu 22.04-et (az alapértelmezett disztribúciót). **Újraindítás szükséges** utána.

Ha kész, nyisd meg az "Ubuntu 22.04" alkalmazást a Start menüből. Első indításkor felhasználónevet és jelszót kér.

### 2. lépés – Grafikus alkalmazások engedélyezése (WSLg)

A Windows 11-ben és a Windows 10 legújabb verzióiban a **WSLg** (WSL GUI) automatikusan működik – a Linux grafikus alkalmazások egyszerűen megjelennek Windows ablakokban.

Ellenőrzés az Ubuntu terminálban:
```bash
echo $DISPLAY
# Valamit kell mutatnia, pl: :0
```

Ha üres, frissítsd a Windows-t.

### 3. lépés – ROS2 Humble telepítése

Az Ubuntu terminálban:

```bash
# UTF-8 locale beállítás (ROS2 megköveteli)
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 GPG kulcs és apt forrás
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
     signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu \
     $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list

# Telepítés
sudo apt update
sudo apt install ros-humble-desktop -y

# Automatikus betöltés minden terminálnyitáskor
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Ellenőrzés:**
```bash
ros2 --version
# ros2 cli libraries: 0.18.x (humble)
```

### 4. lépés – Gazebo Fortress telepítése

```bash
# Gazebo apt forrás
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
     -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
     signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
     http://packages.osrfoundation.org/gazebo/ubuntu-stable \
     $(lsb_release -cs) main" \
     | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update
sudo apt install gz-fortress -y

# ROS2-Gazebo híd
sudo apt install ros-humble-ros-gz -y
```

**Első indítás tesztelése:**
```bash
gz sim shapes.sdf
```
Meg kell jelennie egy 3D-s ablaknak néhány alakzattal. Ha ez működik, a Gazebo telepítve van.

### 5. lépés – Teljesítmény optimalizálás WSL2-ben

A Gazebo 3D szimulációhoz szükség van hardveres GPU gyorsításra. WSL2-ben ez NVIDIA kártya esetén elérhető:

```bash
# NVIDIA driver telepítése (Windows oldalon, nem WSL-ben!)
# Töltsd le a legújabb NVIDIA Game Ready vagy Studio drivert:
# https://www.nvidia.com/Download/index.aspx
# Ez automatikusan telepíti a WSL2 CUDA támogatást is.

# Ellenőrzés WSL2-ben:
nvidia-smi
# Látszania kell a GPU nevének
```

AMD kártyánál a WSL2 GPU gyorsítás korlátozott – ebben az esetben szoftver renderelés:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim shapes.sdf
```

### 6. lépés – A projekt futtatása

```bash
cd /mnt/g/Projects/University/"5. Semester"/Projektmunka/visual_servoing_ws

# Függőségek telepítése
rosdep init  # csak egyszer, ha még nem fut
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Aktiválás
source install/setup.bash

# Indítás!
ros2 launch visual_servoing simulation.launch.py
```

### Windows ↔ Linux fájlmegosztás

A Windows C: meghajtó WSL2-ből elérhető a `/mnt/c/` útvonalon. A projekt a `/mnt/g/`-n van (G: meghajtó). Az Ubuntu fájlrendszer Windows Explorerből elérhető: `\\wsl$\Ubuntu-22.04\`.

---

# 13. Gazebo Windowson – Docker

## Mikor használj Docker-t WSL2 helyett?

- Ha sok projekten dolgozol és izolált környezeteket szeretnél
- Ha a csapattársaid is Docker-t használnak → mindenkinél ugyanaz a konfiguráció
- Ha nem szeretnéd "beszennyezni" a WSL2 Ubuntu-dat package-ekkel

## Docker Desktop telepítése

1. Töltsd le a [Docker Desktop](https://www.docker.com/products/docker-desktop/)-ot
2. Telepítés közben engedélyezd a "Use WSL 2 based engine" opciót
3. Újraindítás után a Docker fut a háttérben

## ROS2 + Gazebo Docker image használata

Az OSRF (Open Source Robotics Foundation) karbantart official image-eket:

```powershell
# PowerShell-ben vagy WSL2 terminálban:
docker pull osrf/ros:humble-desktop
```

Grafikus alkalmazásokhoz (Gazebo ablak megjelenítéséhez) szükséges az X szerver. Windows-on a WSLg automatikusan biztosítja ezt.

**Indítás:**
```bash
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd):/workspace \
  --network host \
  osrf/ros:humble-desktop \
  bash
```

**A paraméterek magyarázata:**
- `--env DISPLAY=$DISPLAY` – a grafikus kijelző átadása
- `--volume /tmp/.X11-unix:/tmp/.X11-unix` – X11 socket megosztás a grafikus kimenethez
- `--volume $(pwd):/workspace` – a jelenlegi mappa látható a konténerben
- `--network host` – ROS2 node-ok elérik egymást a hálózaton

**A Docker módszer hátránya:** Minden indításkor be kell állítani a display forwarding-ot, és a GPU átadás bonyolultabb mint WSL2-ben.

---

# 14. Telepítés és első indítás

## Szükséges csomagok (összefoglaló)

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-ur-description \
  ros-humble-robot-state-publisher \
  python3-colcon-common-extensions \
  python3-rosdep \
  gz-fortress \
  ros-humble-ros-gz

pip install opencv-python numpy pytest
```

## Build és indítás

```bash
cd visual_servoing_ws

# Csak először:
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
# A --symlink-install azt jelenti, hogy Python fájlok
# módosítás után automatikusan érvénybe lépnek (nem kell újrabuildelni)

# Workspace aktiválása (MINDEN új terminálban szükséges)
source install/setup.bash

# Szimuláció indítása
ros2 launch visual_servoing simulation.launch.py
```

## Hasznos megfigyelő parancsok

```bash
# Topic-ok és típusaik listázása
ros2 topic list -t

# Hibavektor valós idejű figyelése
ros2 topic echo /vision/visual_error

# Állapotgép figyelése
ros2 topic echo /assembly/status

# Kamera kép hz mérése (frame rate ellenőrzés)
ros2 topic hz /camera/image_raw

# Debug kép megjelenítése (külön ablakban)
ros2 run rqt_image_view rqt_image_view /vision/debug_image

# Node-ok gráfja (vizualizálja ki kivel kommunikál)
ros2 run rqt_graph rqt_graph
```

---

# 15. Tesztelés és hibakeresés

## Egységtesztek futtatása (ROS2 nélkül)

```bash
cd visual_servoing_ws/src/visual_servoing
pytest test/test_ibvs.py -v

# Várható kimenet:
# PASSED test_zero_error_returns_zero_velocity
# PASSED test_nonzero_error_returns_nonzero_velocity
# PASSED test_velocity_within_limits
# ...
```

## Tipikus hibák és megoldásaik

### "No module named 'rclpy'"
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### "gz: command not found"
```bash
# Gazebo nincs telepítve vagy nem SOURCE-olt
source /opt/ros/humble/setup.bash
```

### A Gazebo ablak nem jelenik meg (WSL2)
```bash
# Ellenőrzés
echo $DISPLAY    # valamit kell mutatnia

# Ha üres:
export DISPLAY=:0
```

### A célpont nem detektálható
Próbáld meg a `canny_low` és `canny_high` értékeket hangolni a `params.yaml`-ban:
```yaml
canny_low: 30    # csökkentsd ha nem talál semmit
canny_high: 100  # csökkentsd ha nem talál semmit
```
Debug képen (`/vision/debug_image`) látható mi detektálódik.

### Az ADRC instabil (rezeg a robot)
Csökkentsd a `control_bandwidth`-ot:
```yaml
control_bandwidth: 5.0   # volt 8.0
```

---

# 16. Irodalom és további olvasnivalók

## Tudományos cikkek

**[1]** Cao C. *Research on a Visual Servoing Control Method Based on Perspective Transformation under Spatial Constraint.* Machines. 2022; 10(11):1090.
→ Ez a projekt fő referenciája. Az IBVS + perspektív transzformáció módszer innen származik.

**[2]** Pomares J. *Visual Servoing in Robotics.* Electronics. 2019; 8(11):1298.
→ Átfogó review cikk az IBVS és PBVS módszerekről.

**[3]** Han J. *From PID to Active Disturbance Rejection Control.* IEEE Transactions on Industrial Electronics. 2009; 56(3):900-906.
→ Az ADRC eredeti cikke Han Jingqing-től.

## Könyvek

**[4]** Corke P. *Robotics, Vision and Control.* Springer, 2022.
→ Legjobb általános robotikai tankönyv, részletes visual servoing fejezettel. Ingyenes PDF: `https://petercorke.com/rvc/`

**[5]** Szeliski R. *Computer Vision: Algorithms and Applications.* Springer, 2022.
→ Homográfia, kamerakalibrálás részletes levezetéssel. Ingyenes: `https://szeliski.org/Book/`

## Online dokumentáció

**[6]** ROS2 Humble dokumentáció: `https://docs.ros.org/en/humble/`

**[7]** OpenCV Python tutorialok: `https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html`
→ Különösen ajánlott: Camera Calibration, Feature Detection fejezetek

**[8]** Gazebo dokumentáció: `https://gazebosim.org/docs/fortress/`

**[9]** MoveIt2 Servo (valós idejű Cartesian vezérlés): `https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html`

**[10]** Universal Robots ROS2 Driver: `https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver`

## Interaktív tanulás

**[11]** The Construct ROS2 kurzusok: `https://www.theconstructsim.com/`
→ Böngészőalapú ROS2 szimulációs környezet, nem kell telepíteni semmit

**[12]** ROS2 hivatalos tutorialok: `https://docs.ros.org/en/humble/Tutorials.html`
→ Ajánlott sorrend: Beginner CLI Tools → Beginner Client Libraries → Intermediate

---

*Dokumentáció vége. Kérdések esetén: ujj.norbert@stud.uni-obuda.hu*
