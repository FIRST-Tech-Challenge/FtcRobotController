package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;

public class RFFlippingIntake extends RFDualServo{
    private RFDualServo flippingIntake;

    LinearOpMode op;

    public RFFlippingIntake (LinearOpMode opMode, String deviceName1, String devicename2, double limit){
        super(opMode, deviceName1, devicename2, limit);

        flippingIntake = new RFDualServo(opMode, deviceName1, devicename2, limit);

        op = opMode;
    }


}
