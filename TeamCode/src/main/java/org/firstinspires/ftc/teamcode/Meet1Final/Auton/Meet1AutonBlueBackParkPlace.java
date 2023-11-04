package org.firstinspires.ftc.teamcode.Meet1Final.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Meet 1 Auton Blue Back Park Place")
public class Meet1AutonBlueBackParkPlace extends LinearOpMode {

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

        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo clawLeft = hardwareMap.servo.get("clawLeft");
        Servo clawRight = hardwareMap.servo.get("clawRight");
        Servo axle = hardwareMap.servo.get("axle");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawLeft.scaleRange(0, 1);
        clawRight.scaleRange(0, 1);

        axle.scaleRange(0, 1);

        int liftTargetPosition = 0;
        double openClaw = 0.4; //increase to open more
        double closeClaw = 0.2;

        waitForStart();

        if (opModeIsActive()) {
            clawLeft.setPosition(closeClaw);
            clawRight.setPosition(closeClaw);
            axle.setPosition(0.8);

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightFrontDrive.setTargetPosition(1470);
            rightFrontDrive.setPower(0.5);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBackDrive.setTargetPosition(1470);
            leftBackDrive.setPower(0.5);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setTargetPosition(-1470);
            leftFrontDrive.setPower(0.5);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightBackDrive.setTargetPosition(-1470);
            rightBackDrive.setPower(0.5);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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


            int leftFrontDriveNecessaryTicks = calculateTicksForLateralMovement(81); //2000
            int rightFrontDriveNecessaryTicks = calculateTicksForLateralMovement(81);
            int leftBackDriveNecessaryTicks = calculateTicksForLateralMovement(81);
            int rightBackDriveNecessaryTicks = calculateTicksForLateralMovement(81);

            int leftFrontDriveTargetTicks =  leftFrontDriveNecessaryTicks;
            int rightFrontDriveTargetTicks = rightFrontDriveNecessaryTicks;
            int leftBackDriveTargetTicks = leftBackDriveNecessaryTicks;
            int rightBackDriveTargetTicks = rightBackDriveNecessaryTicks;

            rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() - leftFrontDriveTargetTicks);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setPower(0.5);

            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - rightFrontDriveTargetTicks);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setPower(0.5);

            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - leftBackDriveTargetTicks);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setPower(0.5);

            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() - rightBackDriveTargetTicks);
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

            for (int liftPos = 0; liftPos <= 500; liftPos = liftPos + 5) {
                liftTargetPosition = liftPos;
                arm.setTargetPosition(liftTargetPosition);
                arm.setPower(0.7);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(15);
            }

            clawLeft.setPosition(openClaw);
            clawRight.setPosition(openClaw);

            sleep(200);
            axle.setPosition(0);
            sleep(1000);

            liftTargetPosition = 10;
            arm.setTargetPosition(liftTargetPosition);
            arm.setPower(0.05);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(500);
            axle.setPosition(0.8);

            while (arm.isBusy()) {}
            arm.setPower(0);


        }

    public int calculateTicksForLateralMovement(double inches) {
        double Calculated_Encoder_Ticks = (inches * Ticks_Per_Inch);
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);
        return Rounded_Encoder_Ticks;
    }
}
