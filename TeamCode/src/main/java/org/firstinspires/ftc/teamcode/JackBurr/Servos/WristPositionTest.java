package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.source.doctree.SerialDataTree;

public class WristPositionTest extends OpMode{
    public Servo wrist_servo;
    @Override
    public void init() {
        wrist_servo = hardwareMap.get(Servo.class, "wrist");
    }

    @Override
    public void loop() {
        telemetry.addData("Position: " , String.valueOf(wrist_servo.getPosition()));
    }
}
