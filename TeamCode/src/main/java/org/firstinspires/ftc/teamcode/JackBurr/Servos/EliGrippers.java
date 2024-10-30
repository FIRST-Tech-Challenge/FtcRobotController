package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class EliGrippers extends OpMode {
    public double pos;
    public Servo grippers;
    public ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        grippers = hardwareMap.get(Servo.class, "grippers");
    }

    @Override
    public void loop() {
        if(gamepad1.x && pos == 1 && timer.seconds()  > 0.3){
            pos = 0;
            timer.reset();
        }
        else if (gamepad1.x && pos == 0 && timer.seconds()  > 0.3){
            pos = 1;
            timer.reset();
        }
        grippers.setPosition(pos);
    }
}
