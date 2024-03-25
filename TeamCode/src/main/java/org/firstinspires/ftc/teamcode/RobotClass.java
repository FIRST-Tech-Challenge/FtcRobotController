package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.AbstractOmniDrivetrain;

public class RobotClass {
    public final AbstractOmniDrivetrain drivetrain;

    HardwareMap hwmap;
    public RobotClass(HardwareMap hwmap){
        this.hwmap = hwmap;
        drivetrain = new MecanumDrive(hwmap.get(DcMotor.class, "frontLeft"),
                                    hwmap.get(DcMotor.class, "frontRight"),
                                    hwmap.get(DcMotor.class, "backLeft"),
                                    hwmap.get(DcMotor.class, "backRight"), 0); // I believe impulse rotation is based on encoder readings
    }


}
