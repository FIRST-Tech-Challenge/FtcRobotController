package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearVerticalLeft {

    public DcMotorEx linear_motion_left;
    HardwareMap hardwareMap;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        linear_motion_left = hardwareMap.get(DcMotorEx.class, "linear_motion_left");
        linear_motion_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear_motion_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
