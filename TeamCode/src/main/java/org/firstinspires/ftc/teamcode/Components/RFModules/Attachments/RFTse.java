package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class RFTse extends RFServo {

    private RFServo tseArmServo;

    LinearOpMode op;
    public RFTse(double range, Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        super(direction, deviceName, opMode);

        tseArmServo = new RFServo(direction, deviceName, opMode);

        op = opMode;
    }
}