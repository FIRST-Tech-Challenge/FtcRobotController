package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

@Config

@Autonomous(name="League1_AutoDriveInchesTest", group ="amogus2")
public class League1_AutoDriveInchesTest extends LinearOpMode {

    public static double driveX = 0.0, driveY = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        waitForStart();

        driveInches(driveX, driveY); // forward 48 inches

        /*while (opModeIsActive()) {

            telemetry.update();
        }*/
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
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
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

    private void stopDriving() {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }
    
    DcMotorEx FL, BL, BR, FR, intakeMotor;
    
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
    }
}