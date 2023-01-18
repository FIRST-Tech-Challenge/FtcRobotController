package org.firstinspires.ftc.teamcode.Helper;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VSlider {
    public DcMotor vSlider;

    private double holdingPower = -0.01;


    /* local OpMode members. */
    //Init hardware map
    HardwareMap hwMap = null;


    //private static LinearOpmode opModeObj;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        vSlider = hwMap.get(DcMotor.class, "vSlider");
        vSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        vSlider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
