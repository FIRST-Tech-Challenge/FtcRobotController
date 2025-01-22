package org.firstinspires.ftc.teamcode.JackBurr.Other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class UnderglowServoClass extends OpMode {
    public Servo underglow;
    @Override
    public void init() {
        underglow = hardwareMap.get(Servo.class, "underglow");
        telemetry.addLine("Press Play (>) to turn underglow on");
    }

    @Override
    public void loop() {
        underglow.setPosition(1);
    }

    @Override
    public void stop(){
        underglow.setPosition(0);
    }

}
