package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;

public class RFFlippingIntake extends RFDualServo{
    private RFDualServo flippingIntake;

    public RFFlippingIntake (String dualServoName, String deviceName1, String devicename2, double limit){
        super(dualServoName, deviceName1, devicename2, limit);

        flippingIntake = new RFDualServo(dualServoName, deviceName1, devicename2, limit);
    }
}
