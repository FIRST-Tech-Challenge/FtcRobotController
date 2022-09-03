package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import static org.firstinspires.ftc.teamcode.BasicRobot.logger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualServo;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class RFFlippingIntake extends RFDualServo{
    private RFDualServo flippingIntake;

    public RFFlippingIntake (String deviceName1, String devicename2, double limit){
        super(deviceName1, devicename2, limit);

        flippingIntake = new RFDualServo(deviceName1, devicename2, limit);
    }


}
