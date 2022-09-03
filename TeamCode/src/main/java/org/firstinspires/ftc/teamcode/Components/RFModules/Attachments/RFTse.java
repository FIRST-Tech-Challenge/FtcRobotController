package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.BasicRobot.logger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFTse extends RFServo {

    private RFServo tseArmServo;

    LinearOpMode op;
    public RFTse(Servo.Direction direction, String deviceName) {
        super(direction, deviceName);

        tseArmServo = new RFServo(direction, deviceName);
    }
}