package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class CappingMechanism {
    private LinearOpMode op = null;
    private Servo capperServo = null;


    public CappingMechanism(LinearOpMode opMode){
        op = opMode;
        capperServo = opMode.hardwareMap.get(Servo.class, "capperServo");
    }

}