package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;

public class RFFlippingIntake extends RFDualServo{
    private RFDualServo flippingIntake;

    LinearOpMode op;

    public RFFlippingIntake (String servoName, String servoName2, Servo.Direction servoDirection, LinearOpMode opMode){
        super(servoName, servoName2, servoDirection, opMode);

        flippingIntake = new RFDualServo(servoName, servoName2, servoDirection, opMode);

        op = opMode;
    }


}
