package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearClaw {

    public DcMotorEx linear_claw;
    HardwareMap hardwareMap;




    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        linear_claw = hardwareMap.get(DcMotorEx.class, "linear_motion_claw");
        linear_claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //linear_claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void loop(){

    }

}
