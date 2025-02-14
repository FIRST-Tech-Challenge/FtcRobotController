package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

@TeleOp
public class IntakeServoTest extends OpMode {
    public Servo servo;
    public int pos;
    public ElapsedTime servoTimer = new ElapsedTime();
    public ElapsedTime buttonTimer = new ElapsedTime();

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "intake_servo");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        servo.setPosition(pos);
        if (gamepad1.b && buttonTimer.seconds() > 0.3 && pos == 0){
            buttonTimer.reset();
            pos = 1;
        }
        else if (gamepad1.b && buttonTimer.seconds() > 0.3 && pos == 1){
            buttonTimer.reset();
            pos = 0;
        }
    }
}

