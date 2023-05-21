package org.firstinspires.ftc.teamcode.TeleOps.AprilTags;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleOp")
public class SampleTeleOp extends OpMode{
    // DECLARING MOTOR VARIABLES
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // DECLARING SERVO OBJECT AND VARIABLES
    private Servo servo;
    private boolean isFlickerDown = true; // flicker should start down
    final double UP = 0.5; // final means this variable cannot be changed
    final double DOWN = 0; // represents the flicker's down position

    @Override
    public void init() {
        // PREPARES SERVO OBJECT
        servo =  hardwareMap.get(Servo.class, "pinballFlicker");

        // PREPARES MOTOR OBJECTS
        frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("BL");

        // SET RUN MODE
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // SET ZERO POWER STATUS
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // REVERSE MOTORS
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        drive();
        flicker();
    }

    public void drive(){
        double leftStickY = -gamepad1.left_stick_y; //forward on the joystick produces a negative y value. Moving right produces a positive x value.
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x * 0.75; // scales down value

        double frontLeftPower = leftStickY + rightStickX;
        double backLeftPower = leftStickY + rightStickX;
        double frontRightPower = leftStickY - rightStickX;
        double backRightPower = leftStickY - rightStickX;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        backLeftMotor.setPower(backLeftPower);
    }

    public void flicker(){
        if(gamepad1.right_bumper){
            if(isFlickerDown) {
                servo.setPosition(UP);
            }
            else{
                servo.setPosition(DOWN);
            }
            isFlickerDown = !isFlickerDown;
        }
    }
}

























