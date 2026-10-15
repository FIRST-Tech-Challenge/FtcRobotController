# KronBot cu NextFTC și Pedro Pathing

Codul de condus folosește NextFTC v2, modulul `dev.nextftc.v2:robot:0.2.1`.
Pedro Pathing rămâne responsabil pentru localizare, condus manual și urmărirea
traseelor. Parametrii mecanici, coordonatele, puterile și formula PID sunt păstrate.

## Cum citești proiectul

Începe cu `AllianceDrivingTeleOp.periodic()`. Metoda descrie, în ordine, un ciclu
complet de control. Metodele apelate de aici separă acțiunile pe responsabilități.

| Clasă | Rol |
| --- | --- |
| `KronBot` | Implementează `NextRobot`, deține follower-ul și mecanismele și gestionează inițializarea și oprirea lor. |
| `DriveController` | Implementează `Mechanism`; controlează deplasarea, field-centric, traseul spre centru și autoaim-ul. |
| `IntakeController` | Implementează `Mechanism`; controlează pornirea, oprirea și inversarea intake-ului. |
| `LimelightController` | Implementează `Mechanism`; inițializează camera, selectează pipeline-ul și afișează rezultatele. |
| `Controls` | Citește gamepad-ul o singură dată pe ciclu și detectează apăsările noi. |
| `AllianceDrivingTeleOp` | Extinde `NextOpMode` și conține comportamentul comun celor două alianțe. |
| `BlueAllianceTeleOp` / `RedAllianceTeleOp` | Aleg numele alianței și coordonatele coșurilor. |
| `utils.Constants` | Conține valorile ajustabile ale comenzilor, pozițiilor și autoaim-ului. |
| `pedroPathing.Constants` | Configurează motoarele, Pinpoint și reglajele follower-ului Pedro. |

## Ce face NextFTC

NextFTC descoperă clasa publică `KronBot`, deoarece aceasta implementează
`NextRobot` și are un constructor public fără argumente. Constructorul creează
obiectele mecanismelor, dar accesarea hardware-ului se face ulterior, la INIT.

Cele două TeleOp-uri au adnotarea `@NextTeleop` și un constructor care primește
`KronBot`. Biblioteca le transmite robotul automat. Nu trebuie să construiești
manual un `BlueAllianceTeleOp` sau un `RedAllianceTeleOp`.

`getMechanisms()` întoarce o colecție stabilă, nemodificabilă, cu mecanismele
robotului. Intake-ul apare în ea numai când este activat. NextFTC cunoaște astfel
mecanismele și le poate gestiona prin infrastructura sa de comenzi.

În această migrare, operațiile existente rămân apeluri directe, în ordinea lor
originală. Nu sunt introduse comenzi asincrone sau legături noi de gamepad, care
ar putea schimba momentul executării acțiunilor. Metodele `periodic()` moștenite
de robot și mecanisme nu fac actualizări suplimentare ale follower-ului.

## Ciclul unui TeleOp

| FTC SDK anterior | NextFTC v2 acum | Responsabilitate |
| --- | --- | --- |
| `init()` | Constructorul `AllianceDrivingTeleOp` | Inițializează follower-ul, țintele, intake-ul opțional și Limelight. |
| `init_loop()` | `disabledPeriodic()` | Permite alegerea poziției de start și actualizează localizarea și telemetria. |
| `start()` | `start()` | Pornește Limelight și permite salvarea poziției. Condusul manual este configurat la inițializarea mecanismului. |
| `loop()` | `periodic()` | Procesează comenzile și actualizează robotul. |
| `stop()` | Hook-ul `afterEnd()` | Oprește deplasarea, intake-ul și camera și eliberează referința follower-ului. |

NextFTC v2 poate reutiliza instanța robotului între rulări. De aceea, inițializarea
resetează starea condusului, țintele și pipeline-ul implicit la fiecare INIT.
`Controls` este creat din nou pentru fiecare TeleOp, astfel încât stările
butoanelor nu se transferă de la o rulare la alta.

`stopRobotOnExit()` creează un `OpModeHook`: un obiect prin care NextFTC anunță
anumite momente din ciclul de viață. Folosim `afterEnd()` deoarece biblioteca îl
apelează din blocul său `finally`, inclusiv dacă o buclă aruncă o excepție.
Constructorul are separat curățare în caz de eroare, deoarece un obiect a cărui
construcție a eșuat nu ajunge să fie gestionat de acel hook.

## Ordinea unei bucle active

1. `controls.update()` copiază starea gamepad-ului și identifică apăsările noi.
2. `handleDrivingButtons()` procesează comutarea field-centric.
3. `updateAutoAim()` pornește sau oprește autoaim-ul în funcție de trigger și modul de condus.
4. `handlePoseReset()` tratează resetarea poziției.
5. `handleIntakeButtons()` tratează intake-ul, dacă este activat.
6. `robot.drive.runManual(controls)` transmite către Pedro deplasarea și rotația dorite, numai în modul manual.
7. `updateAimPipeline()` selectează pipeline-ul coșului folosit pentru autoaim.
8. `robot.update()` apelează exact o dată `follower.update()`.
9. Se afișează telemetria robotului și a camerei.

Actualizarea Pedro rămâne explicită înaintea telemetriei, atât în INIT, cât și în
bucla activă. Nu adăuga încă un `follower.update()` în `KronBot.periodic()` sau
`DriveController.periodic()`: NextFTC ar executa acel apel suplimentar automat.

Pipeline-ul este ales după resetarea poziției și calculul rotației. Astfel, dacă
L3 este apăsat în timpul autoaim-ului, camera folosește coșul corespunzător noii
poziții chiar în acel ciclu.

## Comenzile păstrate

| Control | Acțiune |
| --- | --- |
| Stick stâng | Deplasare înainte/înapoi și lateral, cu deadzone și scalarea existente. |
| Stick drept, axa X | Rotație manuală când autoaim-ul este oprit. |
| Triangle / Y | Comută robot-centric / field-centric, numai în modul manual. |
| Trigger stâng peste 0,5 | Activează autoaim cât timp robotul este în modul manual. La exact 0,5 autoaim-ul este oprit. |
| L3 | Resetează poziția la startul stâng dacă X < 72, altfel la startul drept. |
| D-pad sus / jos în timpul rulării | Nu schimbă pipeline-ul; selecția este automată în autoaim. |
| Bumper dreapta | Comută intake-ul normal, dacă este activat. |
| Bumper stânga | Comută intake-ul inversat, dacă este activat. |
| D-pad stânga / dreapta în INIT | Selectează poziția inițială stângă / dreaptă. |
| D-pad jos în INIT | Încarcă poziția din `PoseStorage`. |

Square și R3 nu au acțiuni asociate în TeleOp-ul existent. Textul vechi din
telemetrie care le atribuia controlul autoaim-ului era depășit; textul a fost
actualizat pentru a descrie comenzile efective.

## Autoaim și valorile ajustabile

Coșul este ales după poziția Y: pentru Y < 72 se folosesc coordonatele coșului 1,
iar pentru Y >= 72 cele ale coșului 2. Alianța stabilește setul de coordonate.
Toate coordonatele de deplasare sunt în sistemul Pedro, în inch.

Pipeline-urile sunt transmise prin constructorul fiecărei alianțe, alături de
coordonatele țintelor:

| Alianță | Y < 72 | Y >= 72 |
| --- | --- | --- |
| Roșie | 1 | 2 |
| Albastră | 3 | 4 |

La INIT, camera primește pipeline-ul primului coș al alianței. În autoaim,
`isUsingFirstAimTarget()` este criteriul comun pentru coordonate și pipeline.
Camera primește o comandă nouă numai când se schimbă numărul pipeline-ului.
La eliberarea triggerului rămâne ultimul pipeline selectat. Valoarea internă
`-1` forțează configurarea la fiecare inițializare și nu este trimisă camerei.
Autoaim-ul continuă să folosească poziția Pedro; rezultatele Limelight sunt
afișate în telemetrie și nu corectează încă rotația.

Expresiile `() -> Constants.BLUE_BASKET_COMMON_X` sunt furnizori de valori
(`DoubleSupplier`). La fiecare citire întorc valoarea curentă a constantei,
permițând reglaje în Panels fără reconstruirea TeleOp-ului. O coordonată care nu
este finită dezactivează autoaim-ul, la fel ca înainte.

Calculul unghiului, normalizarea erorii, deadband-ul, limita integralei și limitarea
puterii la [-1, 1] sunt păstrate. Inclusiv apelarea `startAiming()` în fiecare ciclu
în care triggerul este ținut apăsat, cu resetarea stării PID, este comportament
preexistent. Migrarea nu schimbă această reglare.

API-ul vechi `toggleAimingTarget()` și contorul afișat în câmpul `Aim basket` sunt
păstrate, dar TeleOp-ul nu apelează acea metodă. Contorul nu decide coșul folosit;
coordonatele efective din `Aim coordinates` sunt calculate după Y.

## Intake și utilitare

`KronBot.INTAKE_ENABLED` rămâne `false`. Pentru un robot cu intake montat și motor
configurat ca `intake`, poți schimba valoarea în `true` și reconstrui aplicația.

Utilitarul Pedro `Tuning` rămâne bazat pe `SelectableOpMode`, iar `Logged TeleOp`
rămâne un `OpMode` FTC SDK cu `@PsiKitAutoLog`. Au integrarea proprie cu bibliotecile
lor și pot coexista cu TeleOp-urile NextFTC. Migrarea vizează codul de control
KronBot și nu modifică utilitarele, exemplele SDK sau reglajele Pedro.

## Verificare

Integrarea pipeline-urilor a fost verificată prin construirea aplicației și
42 de verificări cu hardware și ciclu de viață simulate. Acestea acoperă
inițializarea pe ambele alianțe, pragul triggerului, limita Y = 72, resetarea L3
în timpul autoaim-ului, ținte invalide, lipsa comenzilor duplicate către cameră,
păstrarea pipeline-ului la eliberarea triggerului și reinițializarea robotului.
S-a verificat și resetarea indicatorului `canSave` la construirea unui nou TeleOp.

Verificările diferențiale de mai jos descriu migrarea inițială, înaintea
modificărilor ulterioare de comenzi și pipeline-uri.

Aplicația Android a fost construită cu `:TeamCode:assembleDebug` folosind
bibliotecile reale din cache. O verificare diferențială temporară a executat
codul inițial și cel migrat cu aceleași intrări, hardware simulat și ceas
determinist. Au coincis 18.001 evenimente de control și telemetrie: 900 de cicluri
cu intrări pseudoaleatoare reproductibile, plus cazuri explicite, pe ambele
alianțe și la o rulare repetată a alianței albastre. Textele descriptive ale
telemetriei nu au făcut parte din comparație.

Aceleași scenarii au fost comparate și cu intake activat numai în copiile temporare
de test: au coincis încă 19.285 de evenimente, inclusiv la apăsarea simultană a
bumperelor. Au fost verificate separat curățarea la eșecul constructorului,
oprirea în INIT, acțiunea hook-ului de oprire după o eroare de buclă și
reinițializarea aceleiași instanțe `KronBot`. Ciclul NextFTC a fost simulat în
aceste teste; comportamentul hook-urilor a fost verificat și în sursa bibliotecii
0.2.1, nu prin rulare Android pe dispozitiv.

Aceste verificări nu înlocuiesc rularea pe robot. După instalare, verifică apariția
celor două TeleOp-uri pe Driver Station, alegerea poziției în INIT, sensurile de
condus, autoaim-ul în ambele jumătăți ale terenului și oprirea la STOP.

## Documentația bibliotecii

- [NextOpMode și ciclul de viață](https://nextftc.dev/robot/nextopmode/)
- [Descoperirea robotului și a TeleOp-urilor](https://nextftc.dev/robot/project-structure/)
- [Mecanismele NextFTC](https://nextftc.dev/robot/mechanisms/)
