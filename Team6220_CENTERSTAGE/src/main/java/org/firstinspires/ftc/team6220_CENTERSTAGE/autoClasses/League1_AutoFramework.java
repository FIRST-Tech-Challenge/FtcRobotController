package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.team6220_CENTERSTAGE.Utilities;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config

abstract public class League1_AutoFramework extends LinearOpMode {

    public enum AutoAlliance {
        BLUE,
        RED
    }
    public enum AutoType {
        SHORT,
        LONG
    }

    public void doAutoDriveInches(AutoAlliance autoAlliance, AutoType autoType) throws InterruptedException {

        initHardware();

        waitForStart();

        // strafe right on red is +x, -x on blue
        double strafeSignFlip = autoAlliance == AutoAlliance.BLUE ? -1 : 1;

        switch (autoType) {

            case SHORT:

                driveInches(0, 24 * 2); // drive forward to parking spot

                break;

            case LONG:

                sleep(15000); // wait for 15 seconds so alliance an finish

                driveInches(-3 * strafeSignFlip, 0); // strafe away from the wall 3 inches
                driveInches(0, 24 * 4); // drive forward to parking spot

                break;
        }

    }


    static final double TICKS_PER_REVOLUTION = 537.7;
    static final double GEAR_RATIO = 1.0;
    static final double WHEEL_DIAMETER = 3.78; // inches
    static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;

    public static double X_FACTOR = 1.0867924528301887;
    public static double Y_FACTOR = 0.9411764705882353;
    public static double ROBOT_SPEED = 0.5;

    public void driveInches(double x, double y) {
        double xTicks = x * TICKS_PER_INCH * X_FACTOR;
        double yTicks = y * TICKS_PER_INCH * Y_FACTOR;

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
        //runtime.reset();
        FL.setPower(ROBOT_SPEED);
        FR.setPower(ROBOT_SPEED);
        BL.setPower(ROBOT_SPEED);
        BR.setPower(ROBOT_SPEED);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() || isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() && isBusy()) in the loop test.
        while (opModeIsActive() &&
                //(runtime.seconds() < 30) &&
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

    public static double MIN_HEADING_ACCURACY = 5.0; // degrees off from target

    // turn to angle -180 to 180
    public void turnToAngle(double targetHeading) {
        targetHeading = Utilities.limitAngle(targetHeading);

        double turnPower = 0.0;
        double currentHeading = 0.0;

        // turn until roughly the right angle
        while (Math.abs(Utilities.shortestDifference(currentHeading, targetHeading)) > MIN_HEADING_ACCURACY) {

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