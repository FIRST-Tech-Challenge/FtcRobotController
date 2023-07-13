package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Attachments;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class RFIntake extends RFMotor {
    private RFMotor intake;


    public RFIntake(String motorName, DcMotor.RunMode runMode, boolean resetPos) {
        super(motorName, runMode, resetPos);

        intake = new RFMotor(motorName, runMode, resetPos);

    }
}
