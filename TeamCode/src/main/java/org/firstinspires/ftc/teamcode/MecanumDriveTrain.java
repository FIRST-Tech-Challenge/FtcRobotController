package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This file provides necessary functionality for We Love Pi Team's 2019 Rover
 * Ruckus robot's Mecanum Wheel Drivetrain
 */


public class MecanumDriveTrain {

    public double max_stick = 1.0;
    public double min_stick = -1.0;

    // Declare Drive Train motors
    protected DcMotor frontLeft = null;
    protected DcMotor frontRight = null;
    protected DcMotor backLeft = null;
    protected DcMotor backRight = null;
    protected MecanumWheels wheels = new MecanumWheels();


    // Global variables to be initialized in the constructor
    private Telemetry telemetry = null;
    private HardwareMapV2 hardwareMap = null;
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;
    boolean malinDrive = false;
    boolean malinPastState = false;
    double speedlimiter = 1;


    // Constructors

    MecanumDriveTrain(HardwareMapV2 robot, Gamepad gamepadinput, Telemetry telemetry) {
        frontLeft = robot.frontLeft;
        frontRight = robot.frontRight;
        backLeft = robot.backLeft;
        backRight = robot.backRight;
        gamepad1 = gamepadinput;
        this.telemetry = telemetry;

    }


    // During teleop this function runs repeatedly
    public void loop() {

        double left_x = 0.0;
        double left_y = 0.0;
        double right_x = 0.0;
        double left_toggle = 0.55;
        double right_toggle = 0.3;
        double gamepadSum = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;


        // dpad overrides other joysticks
        if (gamepad1.dpad_left) {
            left_x = min_stick;
        } else if (gamepad1.dpad_right) {
            left_x = max_stick;
        } else if (gamepad1.dpad_up) {
            left_y = max_stick;
        } else if (gamepad1.dpad_down) {
            left_y = min_stick;
        }else if(gamepad1.left_trigger >= 0.1) {
            left_x = gamepad1.left_stick_x * left_toggle;
            left_y = -gamepad1.left_stick_y * left_toggle;
            right_x = -gamepad1.right_stick_x * left_toggle;
            max_stick = max_stick * left_toggle;
            min_stick = min_stick * left_toggle;
        }else if(gamepad1.right_trigger >= 0.1){
            left_x = gamepad1.left_stick_x * right_toggle;
            left_y = -gamepad1.left_stick_y * right_toggle;
            right_x = -gamepad1.right_stick_x * right_toggle;
            max_stick = max_stick * right_toggle;
            min_stick = min_stick * right_toggle;
        } else {
            left_x = gamepad1.left_stick_x;
            left_y = -gamepad1.left_stick_y;
            right_x = -gamepad1.right_stick_x;
        }

        if(gamepad1.left_bumper) {
            left_x = -left_x;
        } else {
            left_x = +left_x;
        }

        if (gamepad1.right_bumper && !malinPastState){
            malinDrive = !malinDrive;
        }
        malinPastState = gamepad1.right_bumper;

        if (gamepad1.right_trigger > 0.1){
            speedlimiter = 0.28;
        }else if (gamepad1.left_trigger > 0.1){
            speedlimiter = 0.65;
        }else {
            speedlimiter = 1;
        }

        // Update the joystick input to calculate  wheel powers
        wheels.UpdateInput(left_x, left_y, right_x);

        if (gamepad1.dpad_up) {
            frontLeft.setPower(wheels.getFrontLeftPower());
            frontRight.setPower(wheels.getFrontRightPower());
            backRight.setPower(wheels.getRearRightPower());
            backLeft.setPower(wheels.getRearLeftPower());
        }

        if (!malinDrive){
            frontLeft.setPower(wheels.getFrontLeftPower());
            frontRight.setPower(wheels.getFrontRightPower());
            backRight.setPower(wheels.getRearRightPower());
            backLeft.setPower(wheels.getRearLeftPower());
        }else{
            frontLeft.setPower(-wheels.getRearRightPower());
            frontRight.setPower(-wheels.getRearLeftPower());
            backRight.setPower(-wheels.getFrontLeftPower());
            backLeft.setPower(-wheels.getFrontRightPower());
        }


    }
}