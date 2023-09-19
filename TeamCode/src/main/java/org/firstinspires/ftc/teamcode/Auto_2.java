package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Mecanum - DO")
@Disabled
public class Auto_2 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    public DcMotor frontLeftMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backRightMotor = null;

    public DcMotor liftMotor = null;

    public Servo Claw=null;

    static final double COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 20;   //gear ratio
    static final double WHEEL_DIAMETER_INCH = 96;    // For figuring circumference: 90mm

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH);
    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        forward(.5,10);
        left(.5,10);
    }
    private void forward(double speed , double inches) {
        frontLeftMotor.setTargetPosition((int) (inches * COUNTS_PER_INCH + frontLeftMotor.getCurrentPosition()));
        frontRightMotor.setTargetPosition((int) (inches * COUNTS_PER_INCH + frontRightMotor.getCurrentPosition()));
        backLeftMotor.setTargetPosition((int) (inches * COUNTS_PER_INCH + frontLeftMotor.getCurrentPosition()));
        backRightMotor.setTargetPosition((int) (inches * COUNTS_PER_INCH + frontRightMotor.getCurrentPosition()));

        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && (backLeftMotor.isBusy() || backRightMotor.isBusy())) {
            telemetry.addData("left motor in action", backRightMotor.isBusy());
            telemetry.addData("right motor in action", backLeftMotor.isBusy());
        }
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    private void left(double speed , double inches) {
        frontLeftMotor.setTargetPosition((int) (-inches * COUNTS_PER_INCH + frontLeftMotor.getCurrentPosition()));
        frontRightMotor.setTargetPosition((int) (inches * COUNTS_PER_INCH + frontRightMotor.getCurrentPosition()));
        backLeftMotor.setTargetPosition((int) (inches * COUNTS_PER_INCH + frontLeftMotor.getCurrentPosition()));
        backRightMotor.setTargetPosition((int) (-inches * COUNTS_PER_INCH + frontRightMotor.getCurrentPosition()));

        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && (backLeftMotor.isBusy() || backRightMotor.isBusy())) {
            telemetry.addData("left motor in action", backRightMotor.isBusy());
            telemetry.addData("right motor in action", backLeftMotor.isBusy());
        }
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
