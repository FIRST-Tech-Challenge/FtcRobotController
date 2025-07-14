# FTC Modular Java Library

A reusable, modular Java library for FIRST Tech Challenge (FTC) robots.  
This library helps teams avoid rewriting code every season by providing organized, well-documented classes for hardware, drive systems, manipulators, and autonomous routines.

---

## 📚 What This Library Does

- Provides reusable code for common robot hardware (motors, servos, IMU)
- Supports both robot-centric and field-oriented (IMU-based) driving
- Modular classes for arms, claws, and manipulators
- Utilities for TeleOp and Autonomous modes
- Easy to extend and maintain for future seasons

---

## 🚀 How to Use in Your OpModes

1. **Import the library classes at the top of your OpMode:**
    ```
    import ftc_library.hardware.RobotHardware;
    import ftc_library.drive.FieldOrientedDrive;
    import ftc_library.manipulator.Arm;
    import ftc_library.manipulator.Claw;
    import ftc_library.TeleOpController;
    import ftc_library.autonomous.AutoFarRoutine;
    ```

2. **Initialize hardware and library classes in your OpMode:**
    ```
    RobotHardware hw = new RobotHardware();
    hw.init(hardwareMap);

    FieldOrientedDrive fod = new FieldOrientedDrive(hw);
    Arm arm = new Arm(hw);
    Claw claw = new Claw(hw);
    TeleOpController teleOp = new TeleOpController(fod, arm, claw);
    ```

3. **In your TeleOp loop:**
    ```
    teleOp.handleGamepadInputs(gamepad1, gamepad2);
    ```

4. **In your Autonomous OpMode:**
    ```
    AutoFarRoutine autoFar = new AutoFarRoutine(hw);
    waitForStart();
    autoFar.run();
    ```

---

## 🛠️ How to Extend the Library

- **Add new hardware:**  
  Create new classes in the appropriate package (e.g., `ftc_library.hardware` for sensors, `ftc_library.manipulator` for new mechanisms).
- **Add new drive or manipulator logic:**  
  Extend existing classes or add new ones under `ftc_library.drive` or `ftc_library.manipulator`.
- **Create new autonomous routines:**  
  Add new routines to `ftc_library.autonomous` by extending `BaseAutonomous`.

---

## 📁 Folder Structure

