package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DifferentialArmTest extends OpMode {
    public Servo grippers;
    public Servo arm;
    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        grippers = hardwareMap.get(Servo.class, "grippers");
        arm = hardwareMap.get(Servo.class, "arm");
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0 && timer.seconds() > 0.3){
            if(grippers.getPosition() == 0){
                grippers.setPosition(0.6);
            }
            else{
                grippers.setPosition(0);
            }
            timer.reset();
        }
        if(gamepad1.dpad_left && timer.seconds() > 0.3){
            arm.setPosition(arm.getPosition() - 0.1);
            timer.reset();
        }
        else if(gamepad1.dpad_right && timer.seconds() > 0.3){
            arm.setPosition(arm.getPosition() + 0.1);
            timer.reset();
        }
    }
}
