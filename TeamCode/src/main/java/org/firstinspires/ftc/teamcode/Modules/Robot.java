package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;

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
