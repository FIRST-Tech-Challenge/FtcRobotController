package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive extends AbstractOmniDrivetrain {
    public MecanumDrive(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM){
        super(FLM, FRM, BLM, BRM, Math.PI / 2);
    }

}
