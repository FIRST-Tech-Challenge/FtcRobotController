package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFTse extends RFServo {

    private RFServo tseArmServo;

    LinearOpMode op;
    public RFTse(double range, Servo.Direction direction, String deviceName, LinearOpMode opMode, Logger log) {
        super(range, direction, deviceName, opMode, log);

        tseArmServo = new RFServo(range, direction, deviceName, opMode, log);

        op = opMode;
    }
}