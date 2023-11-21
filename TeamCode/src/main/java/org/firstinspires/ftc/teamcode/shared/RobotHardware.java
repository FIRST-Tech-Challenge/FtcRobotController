package org.firstinspires.ftc.teamcode.shared;


import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.McDonald.VisionLFM2.DEBUG;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
////////////////////////////////////////////////////////////////////////////////////////////////////
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.McDonald.VisionLFM2;

import java.util.Locale;

public class RobotHardware {

    private LinearOpMode myOpMode = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private Servo leftGripper = null;
    private Servo rightGripper = null;
    private Servo wrist = null;
    private DcMotor armMotor = null;

    // Variables

    public static final double FORWARD_SPEED = 0.4;
    public static final double TURN_SPEED = 0.4;
    public static double detectWait = 6.0;
    public static double wristStart = 0.4;
    public static double wristRight = 0;
    public static double wristLeft = 1;
    ////////////////////////////////////////////////////////////////////////////////////////////////
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.778 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    public void init() {

        frontLeftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);


        leftGripper = myOpMode.hardwareMap.get(Servo.class, "leftGripper");
        rightGripper = myOpMode.hardwareMap.get(Servo.class, "rightGripper");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "armMotor");
        wrist = myOpMode.hardwareMap.servo.get("wristServo");
        wrist.setPosition(.4);
        leftGripper.setPosition(1); // Adjust the position value as needed
        rightGripper.setPosition(0); // Adjust the position value as needed

        myOpMode.sleep(1000);

        leftGripper.setPosition(1); // Adjust the position value as needed
        rightGripper.setPosition(0); // Adjust the position value as needed


    }

    public void moveRobot(String path, double time) {

        switch (path.toLowerCase(Locale.ROOT)) {
            case "back":
                frontLeftMotor.setPower(FORWARD_SPEED);
                backLeftMotor.setPower(FORWARD_SPEED);
                frontRightMotor.setPower(FORWARD_SPEED);
                backRightMotor.setPower(FORWARD_SPEED);
                runtime.reset();
                while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
                   // telemetry.addData("Path", "%s: %4.1f S Elapsed", path, runtime.seconds());
                    //telemetry.update();
                }

                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
            default:
                frontLeftMotor.setPower(-FORWARD_SPEED);
                backLeftMotor.setPower(-FORWARD_SPEED);
                frontRightMotor.setPower(-FORWARD_SPEED);
                backRightMotor.setPower(-FORWARD_SPEED);
                runtime.reset();
                while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
                    myOpMode.telemetry.addData("Path", "%s: %4.1f S Elapsed", path, runtime.seconds());
                    myOpMode.telemetry.update();
                }

                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
        }
    }

    public void moveRobot(double speed, double distance, double timeout){

        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newBackRightTarget = backRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        runtime.reset();
        frontRightMotor.setPower(Math.abs(speed));
        frontLeftMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));

        while ((runtime.seconds() < timeout) &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
            myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /////////////////////////////////////////////////lll////////////////////////////////////////////
    private void turnRobot(String direction, double time) {
        switch (direction.toLowerCase(Locale.ROOT)) {
            case "left":
                frontLeftMotor.setPower(TURN_SPEED);
                backLeftMotor.setPower(TURN_SPEED);
                frontRightMotor.setPower(-TURN_SPEED);
                backRightMotor.setPower(-TURN_SPEED);
                runtime.reset();
                //frontLeftMotor.setPower(FORWARD_SPEED);
                //frontRightMotor.setPower(FORWARD_SPEED);
                //backLeftMotor.setPower(FORWARD_SPEED);
                //backRightMotor.setPower(FORWARD_SPEED);

                while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
                    telemetry.addData("Path", "%s: %4.1f S Elapsed", direction, runtime.seconds());
                    telemetry.update();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
            case "right":
                frontLeftMotor.setPower(-TURN_SPEED);
                backLeftMotor.setPower(-TURN_SPEED);
                frontRightMotor.setPower(TURN_SPEED);
                backRightMotor.setPower(TURN_SPEED);
                runtime.reset();
                //frontRightMotor.setPower(TURN_SPEED);
                //backRightMotor.setPower(TURN_SPEED);
                //frontLeftMotor.setPower(-TURN_SPEED);
                //backLeftMotor.setPower(-TURN_SPEED);

                while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
                    //telemetry.addData("Path", "%s: %4.1f S Elapsed", direction, runtime.seconds());
                    //telemetry.update();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;

        }
    }

    // 1 = camera position
    // 2 = pixel drop position

    private void dropProp() {
        telemetry.addData("Dropping Pixel", "");
        telemetry.update();
        leftGripper.setPosition(0.9); // Adjust the position value as needed
        rightGripper.setPosition(0.1); // Adjust the position value as needed
        telemetry.addData("Pixel Dropped", "");
        telemetry.update();
        myOpMode.sleep(1000);

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    private void debugWait() {
        if (DEBUG) {
            sleep(5000);
        } else {
            sleep(1000);
        }
    }

}

    ////////////////////////////////////////////////////////////////////////////////////////////////





