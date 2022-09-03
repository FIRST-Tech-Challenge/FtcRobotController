package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class CappingMechanism {
    private LinearOpMode op = null;
    private Servo capperServo = null;


    public CappingMechanism(){
        capperServo = op.hardwareMap.get(Servo.class, "capperServo");
    }

}