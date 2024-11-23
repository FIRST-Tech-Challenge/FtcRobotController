package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Devices {
        DcMotorEx motorTest;
        DcMotorEx leftFrontDrive;
        DcMotorEx rightFrontDrive;
        DcMotorEx rightBackDrive;
        DcMotorEx leftBackDrive;

    public void init(HardwareMap hwmap) {

        motorTest = hwmap.get(DcMotorEx.class, "motorTest");

        leftFrontDrive = hwmap.get(DcMotorEx.class, "left_front");
        rightFrontDrive = hwmap.get(DcMotorEx.class, "right_front");
        leftBackDrive = hwmap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hwmap.get(DcMotorEx.class, "right_back");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
