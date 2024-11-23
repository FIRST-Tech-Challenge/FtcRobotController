package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Devices {
        DcMotorEx wormGear;
        DcMotorEx leftFrontDrive;
        DcMotorEx rightFrontDrive;
        DcMotorEx rightBackDrive;
        DcMotorEx leftBackDrive;

    public void init(HardwareMap hwmap) {

        wormGear = hwmap.get(DcMotorEx.class, "wormGear");

        leftFrontDrive = hwmap.get(DcMotorEx.class, "leftFront");
        rightFrontDrive = hwmap.get(DcMotorEx.class, "rightFront");
        leftBackDrive = hwmap.get(DcMotorEx.class, "leftBack");
        rightBackDrive = hwmap.get(DcMotorEx.class, "rightBack");

//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
