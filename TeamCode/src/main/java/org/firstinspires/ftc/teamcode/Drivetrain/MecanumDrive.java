package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotClass;

public class MecanumDrive extends AbstractOmniDrivetrain {
    public MecanumDrive(DcMotor FLM, DcMotor BLM, DcMotor FRM, DcMotor BRM, RobotClass robot){
        super(FLM, BLM, FRM, BRM, Math.PI / 2, robot);
    }

}
