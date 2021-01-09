package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "HitterTest")
public class HitterTest extends OpMode {
    //    private SimpleServo servo;
//    private CRServo servo;
    private Motor hitter;
    private boolean pressedUp = false;
    private boolean pressedDown = false;
    private int position = 0;

    @Override
    public void init() {
        hitter = new Motor(hardwareMap, "h");
        hitter.setRunMode(Motor.RunMode.PositionControl);
        hitter.setPositionCoefficient(0.004);
        hitter.setPositionTolerance(13.6);

    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up) {
            pressedUp = true;
        } else if(pressedUp) {
            pressedUp = false;
            position += 5;
        } else if(gamepad1.dpad_down) {
            pressedDown = true;
        } else if(pressedDown) {
            pressedDown = false;
            position -= 5;
        }

        hitter.setTargetPosition(position);
        if(!hitter.atTargetPosition()) {
            hitter.set(0.7);
        } else {
            hitter.set(0);
        }


        telemetry.addData("Hitter Position", hitter.getCurrentPosition());
        telemetry.addData("Position", position);
//        telemetry.addData("Motor Velocity", motor.get());
//        telemetry.addData("Motor Corrected Velocity", motor.getCorrectedVelocity());


    }
}