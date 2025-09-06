package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Slides;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

}
