package org.firstinspires.ftc.teamcode.COMPETITIONCODE.data;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SliderManger {
    private DcMotor rotator; // Change the name
    private DcMotor controller;
    private double pos;
    private double pos2;

    public void reset(){
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init(DcMotor SlideController, DcMotor SlideRotator)
    {
        controller = SlideController;
        rotator = SlideRotator;
        controller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pos = rotator.getCurrentPosition();
        pos2 = controller.getCurrentPosition();
    }
    public void move(double controllerPower){
        controller.setPower(controllerPower);
    }
    public void setPos(int pos){
        rotator.setPower(0.5);
        rotator.setTargetPosition(pos);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setPos2(int pos){
        controller.setPower(0.75);
        controller.setTargetPosition(pos);
        controller.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
