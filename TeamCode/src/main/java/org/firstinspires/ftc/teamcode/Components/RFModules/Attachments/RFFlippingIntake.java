package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.Robot.logger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFFlippingIntake extends RFDualServo{
    private RFDualServo flippingIntake;

    LinearOpMode op;

    public RFFlippingIntake (Servo.Direction servoDirection, LinearOpMode opMode){
        super(servoDirection, opMode);

        flippingIntake = new RFDualServo(servoDirection, opMode);

        op = opMode;
    }


}
