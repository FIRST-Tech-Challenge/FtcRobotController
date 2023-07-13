package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class RFTse extends RFServo {

    private RFServo tseArmServo;

    public RFTse(Servo.Direction direction, String deviceName, double limit) {
        super(deviceName, direction , limit);

        tseArmServo = new RFServo(deviceName, direction , limit);
    }
}