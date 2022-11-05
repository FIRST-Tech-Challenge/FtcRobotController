package org.firstinspires.ftc.team417_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team417_PowerPlay.drive.SampleMecanumDrive;

abstract public class BaseOpMode extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor leftEncoder;
    DcMotor rightEncoder;
    DcMotor frontEncoder;
    DcMotor motorArm;

    Servo grabberServo;
    Toggler grabberToggle;

    // ADJUST THESE VALUES TO CHANGE POSITIONS OF GRABBER
    public static final double GRABBER_OPEN = 0.8;
    public static final double GRABBER_CLOSED = 0.0;

    public void initializeHardware() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorArm = hardwareMap.dcMotor.get("motorArm");

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.FORWARD);

        grabberServo = hardwareMap.servo.get("grabberServo");
        grabberServo.setPosition(GRABBER_CLOSED);
        grabberToggle = new Toggler();

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorArm.setPower(0);
    }

    public void mecanumDrive(double x, double y, double turning) {
        double powerFL;
        double powerFR;
        double powerBL;
        double powerBR;

        powerFL = y + x + turning;
        powerFR = y - x - turning;
        powerBL = y - x + turning;
        powerBR = y + x - turning;

        // THIS IS SO IF ONE OR MORE OF THE POWERS CALCULATED ARE >1.0 IT BECOMES CAPPED AT
        // 1 AND KEEPS PROPORTION TO THE OTHER POWERS
        // COMMENT OUT THIS ENTIRE CODE BLOCK (LINES 95-99) IF WEIRD THINGS HAPPEN
        /*double powerScalar = getLargestValue(powerFL, powerFR, powerBL, powerBR);
        powerFL /= powerScalar;
        powerFR /= powerScalar;
        powerBL /= powerScalar;
        powerBR /= powerScalar;*/

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    public double getLargestValue(double ... nums) {
        double max = Double.MIN_VALUE;
        for (double n : nums) {
            if (n > max) {
                max = n;
            }
        }
        return max;
    }
}
