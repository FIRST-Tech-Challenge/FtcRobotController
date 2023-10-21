package org.firstinspires.ftc.teamcode.versionCode.Auton_2023_10_21;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "V4 Auton No Methods No Exit 10-21-2023")
public class V4_2023_10_21_NoMethodsNoExit extends LinearOpMode {

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

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            int leftFrontDriveNecessaryTicks = calculateTicksForLateralMovement(25); //2000
            int rightFrontDriveNecessaryTicks = calculateTicksForLateralMovement(25);
            int leftBackDriveNecessaryTicks = calculateTicksForLateralMovement(25);
            int rightBackDriveNecessaryTicks = calculateTicksForLateralMovement(25);

            int leftFrontDriveTargetTicks =  leftFrontDriveNecessaryTicks;
            int rightFrontDriveTargetTicks = rightFrontDriveNecessaryTicks;
            int leftBackDriveTargetTicks = leftBackDriveNecessaryTicks;
            int rightBackDriveTargetTicks = rightBackDriveNecessaryTicks;

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + leftFrontDriveTargetTicks);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setPower(0.5);

            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + rightFrontDriveTargetTicks);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setPower(0.5);

            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + leftBackDriveTargetTicks);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setPower(0.5);

            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + rightBackDriveTargetTicks);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setPower(0.5);

            telemetry.addData("Front Left Target Position",leftFrontDrive.getTargetPosition());
            telemetry.addData("Front Left Current Position Position",leftFrontDrive.getCurrentPosition());

            telemetry.addData("Front Right Target Position",rightFrontDrive.getTargetPosition());
            telemetry.addData("Front Right Current Position Position",rightFrontDrive.getCurrentPosition());

            telemetry.addData("Back Left Target Position",leftBackDrive.getTargetPosition());
            telemetry.addData("Back Left Current Position Position",leftBackDrive.getCurrentPosition());

            telemetry.addData("Back Right Target Position",rightBackDrive.getTargetPosition());
            telemetry.addData("Back Right Current Position Position",rightBackDrive.getCurrentPosition());

            telemetry.update();

            while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy()) {}
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);

            int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(12);

            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

            rightFrontDrive.setTargetPosition(Rounded_Encoder_Ticks);
            rightFrontDrive.setPower(0.5);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBackDrive.setTargetPosition(Rounded_Encoder_Ticks);
            leftBackDrive.setPower(0.5);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightBackDrive.setTargetPosition(Rounded_Encoder_Ticks);
            rightBackDrive.setPower(0.5);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setTargetPosition(Rounded_Encoder_Ticks);
            leftFrontDrive.setPower(0.5);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Front Left Target Position",leftFrontDrive.getTargetPosition());
            telemetry.addData("Front Left Current Position Position",leftFrontDrive.getCurrentPosition());

            telemetry.addData("Front Right Target Position",rightFrontDrive.getTargetPosition());
            telemetry.addData("Front Right Current Position Position",rightFrontDrive.getCurrentPosition());

            telemetry.addData("Back Left Target Position",leftBackDrive.getTargetPosition());
            telemetry.addData("Back Left Current Position Position",leftBackDrive.getCurrentPosition());

            telemetry.addData("Back Right Target Position",rightBackDrive.getTargetPosition());
            telemetry.addData("Back Right Current Position Position",rightBackDrive.getCurrentPosition());

            telemetry.update();

            while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }
    }

    public int calculateTicksForLateralMovement(double inches) {
        double Calculated_Encoder_Ticks = (inches * Ticks_Per_Inch);
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);
        return Rounded_Encoder_Ticks;
    }
}
