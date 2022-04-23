package org.firstinspires.ftc.teamcode.Components.RFModules.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class RFIntake extends RFMotor {
    /*all stuff in current intake class minus switch*/
    public RFIntake(String motorName, DcMotorSimple.Direction motorDirection, LinearOpMode op, boolean resetPos) {
        super(motorName,motorDirection,op, DcMotor.RunMode.RUN_WITHOUT_ENCODER,resetPos, DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
