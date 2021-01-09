package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MotorServoTest")
public class MotorServoTest extends OpMode {
    //    private SimpleServo servo;
//    private CRServo servo;
    private Motor motor;
    private boolean aClick = false;
    private boolean shake = true;

    @Override
    public void init() {
        motor = new Motor(hardwareMap, "m");
//        motor = hardwareMap.get(DcMotor.class, "m");
        motor.setRunMode(Motor.RunMode.PositionControl);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPositionCoefficient(0.004);
        motor.setPositionTolerance(13.6);
//        motor = hardwareMap.crservo.get("sv");

    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper) {
            motor.setTargetPosition(240);
            if(!motor.atTargetPosition()) {
                motor.set(0.7);
            } else {
                motor.set(0);
            }
//            motor.set(0.5);
        }
        else {
            motor.setTargetPosition(0);
            if(!motor.atTargetPosition()) {
                motor.set(0.7);
            } else {
                motor.set(0);
            }
        }
//        if(shake) {
//            if(!motor.atTargetPosition()) {
//                motor.set(0.5);
//            } else {
//                motor.stopMotor();
//            }
//
//        } else {
//            motor.stopMotor();
//        }

        if(gamepad1.a) {
            aClick = true;
        } else if(aClick) {
            aClick = false;
            shake = !shake;
        }


        telemetry.addData("Motor Position", motor.getCurrentPosition());
//        telemetry.addData("Motor Velocity", motor.get());
//        telemetry.addData("Motor Corrected Velocity", motor.getCorrectedVelocity());


    }
}