package org.firstinspires.ftc.team6220_2020;

import org.firstinspires.ftc.team6220_2020.ResourceClasses.Button;

public abstract class MasterTeleOp extends MasterOpMode
{

    //Booleans for methods
    boolean justFired = false;
    boolean launcherJustPressed = false;
    public boolean front = true;

    public void driveMecanumWithJoysticks()
    {
        if(front){
            //I negated the inputs to flip the front of the robot
            double turningPower = gamepad1.left_stick_x;
            double driveAngle = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x);
            double drivePower = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
            driveMecanum(driveAngle, drivePower, turningPower);
        } else {
            //I negated the inputs to flip the front of the robot
            double turningPower = gamepad1.left_stick_x;
            double driveAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            double drivePower = Math.hypot(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
            driveMecanum(driveAngle, drivePower, turningPower);
        }
    }

    // Drives launcher with controller
    public void driveLauncherWithController()
    {
        // Todo - migrate to DriverInput class and control to toggle
        driver2.update();

        if (driver2.isButtonJustPressed(Button.A)) {
            launcherJustPressed = !launcherJustPressed;
        }

        if(launcherJustPressed) {
            driveLauncher(1.0);
        }
        else {
            driveLauncher(0.0);
        }
    }

    public void fireLauncherWithTrigger(boolean checkSpeed)
    {
        driver1.update();
        if(!justFired){
            if(driver1.getRightTriggerValue() > 0.7){
                fireLauncher(0);
                justFired = true;
            }
        } else {
            if(driver1.getRightTriggerValue() < 0.3){
                justFired = false;
            }
        }

    }

}

