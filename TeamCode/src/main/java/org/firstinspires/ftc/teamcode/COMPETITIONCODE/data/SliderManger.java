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
        pos = rotator.getCurrentPosition();
        pos2 = controller.getCurrentPosition();
    }
    public void move(double controllerPower, double rotatorPower,boolean run){
        controller.setPower(controllerPower/1.5);
        rotator.setPower(rotatorPower/2 > 0.1 ? rotatorPower/2 - 0.1 : rotatorPower/2);
        if(run){
            pos = rotator.getCurrentPosition();
        }else{
            double currentPos = rotator.getCurrentPosition();
            if(pos - 10.0 >= currentPos || pos + 10.0 <= currentPos) {
                rotator.setPower(pos > currentPos ? 0.35 : -0.35);
            }else{
                rotator.setPower(0.0);
            }
        }
    }
    public void setPos(double pos){
        this.pos = pos;
    }
    public void setPos2(double pos){
        this.pos2 = pos2;
    }
}
