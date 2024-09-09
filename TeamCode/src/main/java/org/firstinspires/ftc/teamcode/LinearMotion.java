package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearMotion {

    public DcMotorEx linear_motion;
    HardwareMap hardwareMap;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        linear_motion = hardwareMap.get(DcMotorEx.class, "linear_motion");
        linear_motion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
