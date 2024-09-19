package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotClass;

import java.util.Map;

public class MecanumDrive extends AbstractOmniDrivetrain {
    public MecanumDrive(Map<RobotClass.MOTORS, DcMotor> drivetrainMecanum, RobotClass robot){
        super(drivetrainMecanum, Math.PI / 2, robot);
    }

}
