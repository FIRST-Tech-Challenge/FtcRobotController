package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;

@Autonomous
public class Auto extends LinearOpMode{

    static final double FEET_PER_METER = 3.28084;

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    public DcMotor liftMotor = null;

    public Servo Claw=null;

    static final double COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 20;   //gear ratio
    static final double WHEEL_DIAMETER_INCH = 3.5;    // For figuring circumference: 90mm

    static final double INCHES_PER_DEGREE = (11.0/90);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * 3.1415);
    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "MotorA");
        rightMotor = hardwareMap.get(DcMotor.class, "MotorB");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Claw = hardwareMap.get(Servo.class, "Servo1");

        liftMotor = hardwareMap.get(DcMotor.class, "MotorC");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        forward(.5,10);
        turn(90,.5);
        lift(.5,6);
    }



    private void forward(double speed , double inches) {



        leftMotor.setTargetPosition((int) (-inches * COUNTS_PER_INCH + leftMotor.getCurrentPosition()));
        rightMotor.setTargetPosition((int) (-inches * COUNTS_PER_INCH + rightMotor.getCurrentPosition()));

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            telemetry.addData("left motor in action", leftMotor.isBusy());
            telemetry.addData("right motor in action", leftMotor.isBusy());
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }
    private void turn(double degrees , double speed) {
        leftMotor.setTargetPosition((int) (degrees * INCHES_PER_DEGREE * COUNTS_PER_INCH + leftMotor.getCurrentPosition()));
        rightMotor.setTargetPosition((int) (-degrees * INCHES_PER_DEGREE * COUNTS_PER_INCH  + rightMotor.getCurrentPosition()));

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            telemetry.addData("left motor in action", leftMotor.isBusy());
            telemetry.addData("right motor in action", rightMotor.isBusy());

        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    private void lift(double speed, double inches) {

        liftMotor.setTargetPosition((int) (inches * INCHES_PER_DEGREE * COUNTS_PER_INCH + leftMotor.getCurrentPosition()));

        liftMotor.setPower(speed);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (liftMotor.isBusy())) {
            telemetry.addData("Motor in use?", liftMotor.isBusy());
        }
        liftMotor.setPower(0);
    }
}

