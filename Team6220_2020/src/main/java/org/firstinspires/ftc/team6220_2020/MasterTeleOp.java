package org.firstinspires.ftc.team6220_2020;

import org.firstinspires.ftc.team6220_2020.ResourceClasses.Button;

public abstract class MasterTeleOp extends MasterOpMode {

    // Booleans for methods
    boolean justFired = false;
    boolean launcherJustPressed = false;
    boolean ziptieJustPressed = false;
    boolean beltJustPressed = false;
    public boolean front = true;

    public void driveMecanumWithJoysticks() {
        if (front) {
            // Negated the inputs to flip the front of the robot
            double turningPower = gamepad1.left_stick_x;
            double driveAngle = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x);
            double drivePower = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
            driveMecanum(driveAngle, drivePower, turningPower);
        }
        else {
            // Negated the inputs to flip the front of the robot
            double turningPower = gamepad1.left_stick_x;
            double driveAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            double drivePower = Math.hypot(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
            driveMecanum(driveAngle, drivePower, turningPower);
        }
    }

    public void driveMecanumWithJoysticksSwitchSticks() {
        if (front) {
            // Negated the inputs to flip the front of the robot
            double turningPower = gamepad1.right_stick_x;
            double driveAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            double drivePower = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            driveMecanum(driveAngle, drivePower, turningPower);
        }
        else {
            // Negated the inputs to flip the front of the robot
            double turningPower = gamepad1.right_stick_x;
            double driveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double drivePower = Math.hypot(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            driveMecanum(driveAngle, drivePower, turningPower);
        }
    }

    // Drives launcher with controller
    public void driveLauncherWithController() {

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

    public void fireLauncherWithTrigger(boolean checkSpeed) {

        if (!justFired) {
            if (driver1.getRightTriggerValue() > 0.7) {
                fireLauncher(0);
                justFired = true;
            }
        }
        else {
            if (driver1.getRightTriggerValue() < 0.3) {
                justFired = false;
            }
        }
    }

    // Drives belt with controller
    public void driveBeltWithController() {

        if (driver2.isButtonJustPressed(Button.B)) {
            beltJustPressed = !beltJustPressed;
        }

        if (driver2.isButtonPressed(Button.RIGHT_BUMPER)) {
            driveBelt(-1.0);
        }
        else if(beltJustPressed){
            driveBelt(1.0);
        } else{
            driveBelt(0.0);
        }
    }

    // Drives zipties with controller
    public void driveZiptiesWithController() {

        if (driver2.isButtonJustPressed(Button.X)) {
            ziptieJustPressed = !ziptieJustPressed;
        }

        if (driver2.isButtonJustPressed(Button.LEFT_BUMPER)) {
            driveZiptie(-1.0);
        }
        else if(ziptieJustPressed){
            driveZiptie(1.0);
        }
        else {
            driveZiptie(0.0);
        }
    }

    public void shootPowerShot(){
        driveInches(180, 26, 0.5);

        driveLauncher(0.94);

        pauseMillis(1500);

        for(int i = 0; i < 3; i++){
            fireLauncher(0);
            driveInches(7.5,0, 0.2);
            pauseMillis(750);
        }

        driveLauncher(0.0);
    }
}