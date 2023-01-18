package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwingArm {
    public DcMotor swingArm;
    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        swingArm = hwMap.get(DcMotor.class, "swingArm");
        swingArm.setDirection(DcMotorSimple.Direction.FORWARD);
        swingArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swingArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
