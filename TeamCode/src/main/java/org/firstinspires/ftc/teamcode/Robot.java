package org.firstinspires.ftc.teamcode;

public class Robot {
    private MecanumDrive driveSystem;
    private JoystickController joystickController;

    public Robot(MecanumDrive driveSystem, JoystickController joystickController) {
        this.driveSystem = driveSystem;
        this.joystickController = joystickController;
    }

    public void drive(double forward, double strafe, double rotate) {
        driveSystem.drive(forward, strafe, rotate);
    }

    public void updateJoystickControl() {
        joystickController.update();
    }
}
