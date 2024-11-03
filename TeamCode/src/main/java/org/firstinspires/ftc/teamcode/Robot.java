package org.firstinspires.ftc.teamcode;

class Robot {
    private MecanumDrive driveSystem;
    private JoystickController joystickController;

    // Constructor to initialize the robot subsystems
    public Robot(MecanumDrive driveSystem, JoystickController joystickController) {
        this.driveSystem = driveSystem;
        this.joystickController = joystickController;
        System.out.println("Robot initialized");
    }

    public MecanumDrive getDriveSystem() {
        return driveSystem;
    }

    // Drive method for autonomous use
    public void drive(double forward, double strafe, double rotate) {
        driveSystem.drive(forward, strafe, rotate);
        System.out.println(String.format("Autonomous drive - Forward: %2.2f, Strafe: %2.2f, Rotate: %2.2f", forward, strafe, rotate));
    }

    // Update joystick control for teleop use
    public void updateJoystickControl() {
        if (joystickController != null) {
            joystickController.update();
        }
    }
}