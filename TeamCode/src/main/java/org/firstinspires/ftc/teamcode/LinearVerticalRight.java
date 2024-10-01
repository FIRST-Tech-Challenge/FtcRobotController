package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearVerticalRight {

    public DcMotorEx linear_motion_right;
    HardwareMap hardwareMap;



    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        linear_motion_right = hardwareMap.get(DcMotorEx.class, "linear_motion_right");
        linear_motion_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linear_motion_right.setDirection(DcMotorEx.Direction.REVERSE);

        //linear_motion_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






    }
}
