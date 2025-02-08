package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmServoChange extends OpMode {
    public Servo axon;
    @Override
    public void init() {
        axon = hardwareMap.get(Servo.class, "deliveryServo");
    }

    @Override
    public void loop() {
        axon.setPosition(0.48);
    }
}
