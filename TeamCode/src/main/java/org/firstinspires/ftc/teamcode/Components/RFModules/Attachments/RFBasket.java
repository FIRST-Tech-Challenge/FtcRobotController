package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class RFBasket extends RFServo {

    private RFServo basketServo;

    LinearOpMode op;
    public RFBasket(double range, Servo.Direction direction, String deviceName, LinearOpMode opMode) {
        super(range, direction, deviceName, opMode);

        basketServo = new RFServo(range, direction, deviceName, opMode);

        op = opMode;
    }
}