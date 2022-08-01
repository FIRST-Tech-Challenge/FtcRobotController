package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFFlippingIntake extends RFDualServo{
    private RFDualServo flippingIntake;

    LinearOpMode op;

    public RFFlippingIntake (Servo.Direction servoDirection, LinearOpMode opMode, Logger log){
        super(servoDirection, opMode, log);

        flippingIntake = new RFDualServo(servoDirection, opMode, log);

        op = opMode;
    }


}
