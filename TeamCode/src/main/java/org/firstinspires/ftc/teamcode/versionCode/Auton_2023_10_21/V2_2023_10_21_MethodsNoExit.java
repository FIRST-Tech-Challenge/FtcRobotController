package org.firstinspires.ftc.teamcode.versionCode.Auton_2023_10_21;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "V2 Auton Methods No Exit 10-21-2023")
public class V2_2023_10_21_MethodsNoExit extends LinearOpMode {

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private double Ticks_Per_Inch = 45.2763982107824;

    private int leftFrontDriveTickTracker = 0;
    private int rightFrontDriveTickTracker = 0;
    private int leftBackDriveTickTracker = 0;
    private int rightBackDriveTickTracker = 0;

    @Override
    public void runOpMode() {

        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("motorFL");
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("motorBL");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("motorFR");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("motorBR");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        ResetEncoders();
        waitForStart();

        while (opModeIsActive()) {
            //moveForward(0.5, 25);
            //moveBackward(0.5, 10);
            //moveLeft(0.5, 10);
            //moveRight(0.5, 10);
            //rotateClockwise(0.5, 3000);
            moveToPosition(0.5, 25);
            moveRight(0.5, 12);
        }
    }

    public void moveToPosition(double speed, double inches) {
        int leftFrontDriveNecessaryTicks = calculateTicksForLateralMovement(inches); //2000
        int rightFrontDriveNecessaryTicks = calculateTicksForLateralMovement(inches);
        int leftBackDriveNecessaryTicks = calculateTicksForLateralMovement(inches);
        int rightBackDriveNecessaryTicks = calculateTicksForLateralMovement(inches);

        int leftFrontDriveTargetTicks =  leftFrontDriveNecessaryTicks;
        int rightFrontDriveTargetTicks = rightFrontDriveNecessaryTicks;
        int leftBackDriveTargetTicks = leftBackDriveNecessaryTicks;
        int rightBackDriveTargetTicks = rightBackDriveNecessaryTicks;

        SetFrontLeftDriveDirection("forward");
        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("forward");
        SetBackRightDriveDirection("forward");

        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + leftFrontDriveTargetTicks);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(speed);

        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + rightFrontDriveTargetTicks);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(speed);

        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + leftBackDriveTargetTicks);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);

        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + rightBackDriveTargetTicks);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(speed);

        motionTelemetry();

        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy()) {}
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
    }

    public void moveRight(double speed, double inches) {

        int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(inches);

        SetFrontRightDriveDirection("backward");
        SetBackLeftDriveDirection("backward");

        rightFrontDrive.setTargetPosition(Rounded_Encoder_Ticks);
        rightFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(Rounded_Encoder_Ticks);
        leftBackDrive.setPower(speed);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motionTelemetry();

        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy()) {}
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
    }

    public void SetFrontLeftDriveDirection(String direction) {
        if(direction == "forward") {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (direction == "backward") {
            leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void SetFrontRightDriveDirection(String direction) {
        if(direction == "forward") {
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == "backward") {
            rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void SetBackLeftDriveDirection(String direction) {
        if(direction == "forward") {
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        } else if (direction == "backward") {
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void SetBackRightDriveDirection(String direction) {
        if(direction == "forward") {
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == "backward") {
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void ResetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForward(double speed, double inches) {

        int leftFrontDriveNecessaryTicks = calculateTicksForLateralMovement(inches); //2000
        int rightFrontDriveNecessaryTicks = calculateTicksForLateralMovement(inches);
        int leftBackDriveNecessaryTicks = calculateTicksForLateralMovement(inches);
        int rightBackDriveNecessaryTicks = calculateTicksForLateralMovement(inches);


        int leftFrontDriveCurrentTicks = leftFrontDrive.getCurrentPosition();
        int rightFrontDriveCurrentTicks = rightFrontDrive.getCurrentPosition();
        int leftBackDriveCurrentTicks = leftBackDrive.getCurrentPosition();
        int rightBackDriveCurrentTicks = rightBackDrive.getCurrentPosition();

        int leftFrontDriveTargetTicks =  leftFrontDriveNecessaryTicks;
        int rightFrontDriveTargetTicks = rightFrontDriveNecessaryTicks;
        int leftBackDriveTargetTicks = leftBackDriveNecessaryTicks;
        int rightBackDriveTargetTicks = rightBackDriveNecessaryTicks;

        SetFrontLeftDriveDirection("forward");
        SetFrontRightDriveDirection("forward");
        SetBackLeftDriveDirection("forward");
        SetBackRightDriveDirection("forward");

        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + leftFrontDriveTargetTicks);
        leftFrontDrive.setPower(speed);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setTargetPosition(rightFrontDriveTargetTicks);
        rightFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(leftBackDriveTargetTicks);
        leftBackDrive.setPower(speed);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setTargetPosition(rightBackDriveTargetTicks);
        rightBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motionTelemetry();
    }

    public int calculateTicksForLateralMovement(double inches) {
        double Calculated_Encoder_Ticks = (inches * Ticks_Per_Inch);
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);
        return Rounded_Encoder_Ticks;
    }

    public void motionTelemetry() {
        telemetry.addData("Front Left Target Position",leftFrontDrive.getTargetPosition());
        telemetry.addData("Front Left Current Position Position",leftFrontDrive.getCurrentPosition());

        telemetry.addData("Front Right Target Position",rightFrontDrive.getTargetPosition());
        telemetry.addData("Front Right Current Position Position",rightFrontDrive.getCurrentPosition());

        telemetry.addData("Back Left Target Position",leftBackDrive.getTargetPosition());
        telemetry.addData("Back Left Current Position Position",leftBackDrive.getCurrentPosition());

        telemetry.addData("Back Right Target Position",rightBackDrive.getTargetPosition());
        telemetry.addData("Back Right Current Position Position",rightBackDrive.getCurrentPosition());

        telemetry.update();
    }

}   // end class

