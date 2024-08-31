package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive extends AbstractOmniDrivetrain {
    public MecanumDrive(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, HardwareMap hwmap){
        super(FLM, FRM, BLM, BRM, Math.PI / 2, hwmap);
    }

}
