package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.team6220_CENTERSTAGE.Utilities;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config

abstract public class League1_AutoFramework extends LinearOpMode {

    // used in determining which alliance side to use in auto path
    public enum AutoAlliance {
        BLUE,
        RED,
    }
    // used in determining which side of the truss to use in auto path
    // name represents path length to backstage
    public enum AutoStartingPositionType {
        SHORT,
        LONG,
    }

    // run the auto path determined by the chosen starting position details
    public void runAutoFramework(AutoAlliance autoAlliance, AutoStartingPositionType autoStartingPositionType) throws InterruptedException {

        initHardware();

        waitForStart();

        // strafe right on red is +x, -x on blue
        double strafeSignFlip = autoAlliance == AutoAlliance.BLUE ? -1 : 1;

        switch (autoStartingPositionType) {

            case SHORT:

                driveInches(0, -24 * 2); // drive forward to parking spot

                break;

            case LONG:

                sleep(15000); // wait for 15 seconds so alliance an finish

                driveInches(3 * strafeSignFlip, 0); // strafe away from the wall 3 inches
                driveInches(0, -24 * 4); // drive forward to parking spot

                break;
        }

    }



    public void driveInches(double x, double y) {
        double xTicks = x * Constants.TICKS_PER_INCH * Constants.AUTO_X_FACTOR;
        double yTicks = y * Constants.TICKS_PER_INCH * Constants.AUTO_Y_FACTOR;

        double targetFL = xTicks + yTicks;
        double targetFR = yTicks - xTicks;
        double targetBL = yTicks - xTicks;
        double targetBR = yTicks + xTicks;

        // Determine new target position, and pass to motor controller
        targetFL += FL.getCurrentPosition();
        targetFR += FR.getCurrentPosition();
        targetBL += BL.getCurrentPosition();
        targetBR += BR.getCurrentPosition();

        FL.setTargetPosition((int) targetFL);
        FR.setTargetPosition((int) targetFR);
        BL.setTargetPosition((int) targetBL);
        BR.setTargetPosition((int) targetBR);

        // Turn On RUN_TO_POSITION
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        FL.setPower(Constants.ROBOT_AUTO_SPEED);
        FR.setPower(Constants.ROBOT_AUTO_SPEED);
        BL.setPower(Constants.ROBOT_AUTO_SPEED);
        BR.setPower(Constants.ROBOT_AUTO_SPEED);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() || isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() && isBusy()) in the loop test.
        while (opModeIsActive() &&
                (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
        }

        // Stop all motion;
        stopDriving();

        // Turn off RUN_TO_POSITION
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // turn to angle -180 to 180
    public void turnToAngle(double targetHeading) {
        targetHeading = Utilities.limitAngle(targetHeading);

        double turnPower = 0.0;
        double currentHeading = 0.0;

        // turn until roughly the right angle
        while (Math.abs(Utilities.shortestDifference(currentHeading, targetHeading)) > Constants.AUTO_MIN_HEADING_ACCURACY) {

            // get heading from imu in degrees
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            turnPower = Utilities.clamp(-Utilities.shortestDifference(currentHeading, targetHeading) / 90.0);

            turnInPlaceWithPower(turnPower);

            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.addData("turnPower", turnPower);
            telemetry.update();

        }

        stopDriving();
    }

    public void turnInPlaceWithPower(double turnPower) {
        FL.setPower(turnPower);
        FR.setPower(-turnPower);
        BL.setPower(turnPower);
        BR.setPower(-turnPower);
    }

    private void stopDriving() {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }
    
    DcMotorEx FL, BL, BR, FR, intakeMotor;
    IMU imu;
    
    public void initHardware() {
        // stolen from roadrunner mecanum drive class
        FL = hardwareMap.get(DcMotorEx.class, "motFL");
        BL = hardwareMap.get(DcMotorEx.class, "motBL");
        BR = hardwareMap.get(DcMotorEx.class, "motBR");
        FR = hardwareMap.get(DcMotorEx.class, "motFR");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // reverse fr and br motors so that it drives correctly
        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // now has been enabled, encoders are goodge :D
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
}