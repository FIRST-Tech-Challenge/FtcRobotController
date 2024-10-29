package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class EliIntake extends OpMode {
    public Servo leftWheel;
    public Servo rightWheel;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public enum WheelDirection {
        FORWARD,
        BACKWARD
    }
    public WheelDirection wheelDirection;

    @Override
    public void init() {
        leftWheel = hardwareMap.get(Servo.class, "leftWheel");
        rightWheel = hardwareMap.get(Servo.class, "rightWheel");
        wheelDirection = WheelDirection.FORWARD;
        leftWheel.setPosition(0);
        rightWheel.setPosition(1);
        buttonTimer.reset();
    }

    @Override
    public void loop() {
        if(gamepad1.x && buttonTimer.seconds() > 0.3) {
            if (wheelDirection == WheelDirection.FORWARD) {
                leftWheel.setPosition(0);
                rightWheel.setPosition(1);
            } else {
                leftWheel.setPosition(1);
                rightWheel.setPosition(0);
            }
            buttonTimer.reset();
        }
    }
}
