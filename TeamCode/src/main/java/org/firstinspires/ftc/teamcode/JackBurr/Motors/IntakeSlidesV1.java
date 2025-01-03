package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class IntakeSlidesV1 {
    public HardwareMap hardwareMap;
    public DcMotor intakeSlides;
    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.intakeSlides = hardwareMap.get(DcMotor.class,"intakeSlides");
        intakeSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlides.setPower(0);
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runToPosition(int position, double power){
        if(intakeSlides.getCurrentPosition() != position) {
            intakeSlides.setPower(power);
            intakeSlides.setTargetPosition(position);
            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            intakeSlides.setPower(0);
        }
    }
}
