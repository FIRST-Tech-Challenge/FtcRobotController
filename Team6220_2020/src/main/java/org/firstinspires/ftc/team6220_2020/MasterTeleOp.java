package org.firstinspires.ftc.team6220_2020;

public abstract class MasterTeleOp extends MasterOpMode
{
    public void driveMecanumWithJoysticks()
    {
        double turningPower = gamepad1.left_stick_x;
        double driveAngle = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x);
        double drivePower = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
        driveMecanum(driveAngle, drivePower, turningPower);
    }

    // Drives launcher with controller. X starts launcher Y stops launcher
    public void driveLauncherWithController()
    {
        // Todo - migrate to DriverInput class and control to toggle
        if (gamepad2.x) {
            driveLauncher(1.0);
        }
        else if(gamepad2.y){
            driveLauncher(0.0);
        }
    }
}

