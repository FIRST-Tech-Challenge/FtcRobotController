package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team417_2020.Resources.Toggler;

abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;
    private boolean openWobbleGoalGrabber = false;
    private boolean upWobbleGoalArm = false;
    private boolean midWobbleGoalArm = false;

    private Toggler slowMode = new Toggler();
    private Toggler reverseMode = new Toggler();
    private Toggler wobbleGoalGrabberToggler = new Toggler();
    private Toggler wobbleGoalArmTogglerUpDown = new Toggler();
    private Toggler wobbleGoalArmTogglerMid = new Toggler();

    int targetPosition = 0;


    /**
     * Uses the mecanum drive function to move the robot | Right stick translate, Left stick rotate (gamepad1)
     */
    public void driveRobot()
    {
        isSlowMode = slowMode.toggle(gamepad1.right_bumper);
        isReverseMode = reverseMode.toggle(gamepad1.left_bumper);

        double y = -gamepad1.right_stick_y; // Y is negative above the Y axis
        double x = gamepad1.right_stick_x;
        double rotationalPower = gamepad1.left_stick_x;

        if (isSlowMode) {
            y *= 0.2;
            x *= 0.2;
            rotationalPower *= 0.2;
        }
        else if (isReverseMode) {
            y *= -1;
            x *= -1;
        }


        // todo check and test to see if we need filtering
        /*
        filterJoyStickInput.appendInput(x, y, pivotPower);

        x = filterJoyStickInput.getFilteredX();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();
         */

        double drivePower = Math.hypot(x, y);
        double angle = Math.atan2(y, x);

        mecanumDrive(angle, drivePower, rotationalPower);
    }

    public void setWobbleGoalGrabber() {

        openWobbleGoalGrabber = wobbleGoalGrabberToggler.toggle(gamepad1.a);
        upWobbleGoalArm = wobbleGoalArmTogglerUpDown.toggle(gamepad1.right_bumper);
        midWobbleGoalArm = wobbleGoalArmTogglerMid.toggle(gamepad1.left_bumper);
        if (openWobbleGoalGrabber) {
            wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_OUT);
        } else {
            wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_IN);
        }

        if (gamepad1.b) {
            targetPosition += 10;
            sleep(100);
        }

        if (!midWobbleGoalArm) {
            if (upWobbleGoalArm) {
                runMotorToPosition(motorWobbleGoalArm, WOBBLE_GOAL_ARM_UP, 0.2);
            } else {
                runMotorToPosition(motorWobbleGoalArm, WOBBLE_GOAL_ARM_DOWN, 0.2);
            }

        } else {
            runMotorToPosition(motorWobbleGoalArm, WOBBLE_GOAL_ARM_MID, 0.2);
        }


        telemetry.addData("Wobble Goal Open", openWobbleGoalGrabber);
        telemetry.addData("Target Position", targetPosition);


    }

    public void moveWobbleGoalArm() {

        openWobbleGoalGrabber = wobbleGoalGrabberToggler.toggle(gamepad2.a);
        upWobbleGoalArm = wobbleGoalArmTogglerUpDown.toggle(gamepad2.right_bumper);
        midWobbleGoalArm = wobbleGoalArmTogglerMid.toggle(gamepad2.left_bumper);
        if (openWobbleGoalGrabber) {
            wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_OUT);
        } else {
            wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_IN);
        }

        motorWobbleGoalArm.setPower(gamepad2.left_stick_y * -0.5);
        if (gamepad1.left_stick_y == 0) {
            motorWobbleGoalArm.setPower(0.0);
        }

    }

}
