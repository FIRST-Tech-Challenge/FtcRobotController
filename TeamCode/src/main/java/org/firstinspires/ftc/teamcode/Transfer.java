package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Transfer {
    private final Servo transferServo;



    public Transfer(OpMode opMode) {
        transferServo = opMode.hardwareMap.servo.get("transferServo");

    }
}
