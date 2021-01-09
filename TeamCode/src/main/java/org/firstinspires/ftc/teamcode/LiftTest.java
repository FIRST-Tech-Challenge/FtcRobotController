package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LiftTest")
public class LiftTest extends OpMode {
    //    private SimpleServo servo;
    private Servo servo;
//    private CRServo servo;
//    private Motor motor;

    double multiplier = 1;

    @Override
    public void init() {
//        motor = new Motor(hardwareMap, "m");
//        motor.setRunMode(Motor.RunMode.RawPower);

        servo = hardwareMap.servo.get("sv");

    }

    @Override
    public void loop() {
//        motor.set(1);
        if(gamepad1.a) {
//            servo.setPosition(-0.5);
            multiplier = 1;
//            servo.setDirection(DcMotorSimple.Direction.FORWARD);
//            servo.setPower(.5);
//            servo.rotate(3);
//            servo.
        } else if(gamepad1.b){
            multiplier = 2;
        } else if(gamepad1.x) {
            multiplier = 1.5;
        } else if(gamepad1.y) {
            multiplier = 3;
        }
        else {
//            servo.set(gamepad1.left_stick_x);

        }

        servo.setPosition(gamepad1.left_stick_x * multiplier);

        telemetry.addData("Servo Power", servo.getPosition());
//        telemetry.addData("Servo Position", servo.getCurrentPosition());
//        telemetry.addData("Servo Velocity", servo.getCorrectedVelocity());
//        telemetry.addData("Servo CPR", servo.getCPR());


    }
}