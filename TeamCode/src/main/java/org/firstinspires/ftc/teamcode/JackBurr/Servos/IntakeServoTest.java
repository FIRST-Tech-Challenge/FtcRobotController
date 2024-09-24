package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class IntakeServoTest extends OpMode {
    public Servo servo;
    public ElapsedTime servoTimer = new ElapsedTime();
    public ElapsedTime buttonTimer = new ElapsedTime();
    @Override
    public void init() {
        servo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        servo.setPosition(1);
        if (gamepad1.b && buttonTimer.seconds() > 0.3){
            buttonTimer.reset();
            servoTimer.reset();
            servo.setPosition(0);
            while (servoTimer.seconds() < 0.7){
                //wait
            }
            servo.setPosition(1);
        }
    }
}
