# Mecanismele FTC_TestRobot fără NextFTC

Codul din `biobuzz` folosește din nou FTC SDK (`OpMode`) și Pedro Pathing direct.
Structura a fost refăcută după ultimul snapshot complet anterior migrării NextFTC din
`FTC_TestRobot`: commitul `dc80737`.

Commitul literal `HEAD~1`, `c7572de`, este o migrare NextFTC incompletă: `Robot.java`
importă `mechanisms/Drive.java` și `mechanisms/Intake.java`, dar acele fișiere nu există
în acel commit. Din acest motiv nu poate fi sursa unei implementări fără NextFTC.

## Structură

- `KronBot` construiește follower-ul Pedro și deține mecanismele.
- `DriveController` controlează condusul manual, field-centric, autoaim și traseul spre centru.
- `IntakeController` controlează motorul configurat cu numele `intake`.
- `Controls` detectează apăsarea inițială a butoanelor fără un scheduler extern.
- `AllianceDrivingTeleOp` conține comportamentul comun.
- `BlueAllianceTeleOp` poate selecta numai cele două coșuri albastre.
- `RedAllianceTeleOp` poate selecta numai cele două coșuri roșii.

Intake-ul este prezent în codebase, dar `KronBot.INTAKE_ENABLED` este `false`, la fel ca
în snapshotul sursă. Dacă mecanismul este montat și există dispozitivul `intake` în
configurația Robot Controller, valoarea poate fi schimbată în `true`.

## Comenzi

| Buton | Acțiune |
| --- | --- |
| Cross / A | Merge spre centrul terenului dacă autoaim și field-centric sunt oprite |
| Circle / B | Revine la condus manual |
| Triangle / Y | Comută robot-centric / field-centric |
| Square / X | Activează / dezactivează autoaim |
| R3 | Schimbă între coșul 1 și coșul 2 al alianței selectate |
| L3 | Resetează poziția la startul din partea curentă |
| D-pad sus / jos | Pipeline Limelight 3 / 4 |
| Bumper dreapta / stânga | Intake normal / inversat, dacă este activat |

În INIT, D-pad stânga/dreapta alege poziția inițială. Alegerea trebuie să corespundă
poziției fizice a robotului.

## Fluxul unei bucle

1. `Controls.update()` copiază starea gamepad-ului și detectează fronturile butoanelor.
2. Butoanele schimbă starea mecanismelor.
3. `DriveController.runManual()` transmite comanda de condus către Pedro.
4. `KronBot.update()` rulează `follower.update()`, care actualizează localizarea și motoarele.
5. Telemetria drivetrain-ului și Limelight este trimisă către Driver Station.

Toate coordonatele sunt în sistemul Pedro și în inch. PID-ul autoaim și coordonatele
țintelor trebuie verificate și reglate pe robotul fizic.

Cele patru perechi de coordonate sunt intenționat `Double.NaN` în `Constants.java`:

- `BLUE_BASKET_1_X`, `BLUE_BASKET_1_Y`
- `BLUE_BASKET_2_X`, `BLUE_BASKET_2_Y`
- `RED_BASKET_1_X`, `RED_BASKET_1_Y`
- `RED_BASKET_2_X`, `RED_BASKET_2_Y`

Autoaim-ul refuză să pornească pentru un coș până când ambele coordonate ale acelui
coș sunt numere finite. Valorile sunt citite dinamic, deci pot fi ajustate și prin
configurarea Panels fără reconstruirea obiectului OpMode.
