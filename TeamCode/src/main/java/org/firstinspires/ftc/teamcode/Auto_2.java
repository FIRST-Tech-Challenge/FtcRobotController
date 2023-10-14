package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Auto_2 extends LinearOpMode{
    static final double FEET_PER_METER = 3.28084;

    public DcMotor backLeftMotor = null;

    public DcMotor frontLeftMotor = null;

    public DcMotor backRightMotor = null;

    public DcMotor frontRightMotor = null;

    public DcMotor liftMotor = null;

    public DcMotor armMotor = null;

    static final double COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 19.2;   //gear ratio

    static final double ARM_GEAR_REDUCTION = 50.9; //gear ratio
    static final double WHEEL_DIAMETER_INCH = 3.5;    // For figuring circumference: 90mm

    static final double INCHES_PER_DEGREE = (11.0 / 90);

    static final double COUNTS_PER_INCH2 = (COUNTS_PER_MOTOR_REV * ARM_GEAR_REDUCTION) ;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * 3.1415);
    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        forward(.5,10);
        lift(90,0.5);

    }

    private void forward(double speed, double inches){

        frontRightMotor.setTargetPosition((int)(-inches * COUNTS_PER_INCH + frontRightMotor.getCurrentPosition()));
        frontLeftMotor.setTargetPosition((int)(-inches * COUNTS_PER_INCH + frontLeftMotor.getCurrentPosition()));
        backRightMotor.setTargetPosition((int)(-inches * COUNTS_PER_INCH + backRightMotor.getCurrentPosition()));
        backLeftMotor.setTargetPosition((int)(-inches * COUNTS_PER_INCH + backLeftMotor.getCurrentPosition()));

        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (backLeftMotor.isBusy() || backRightMotor.isBusy())) {

            }
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

    }
    private void lift(double degrees, double speed) {
        liftMotor.setTargetPosition((int)(degrees * INCHES_PER_DEGREE * COUNTS_PER_INCH2 + liftMotor.getCurrentPosition()));

        liftMotor.setPower(speed);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (liftMotor.isBusy())){
        }

        liftMotor.setPower(0);
    }

}        
        

