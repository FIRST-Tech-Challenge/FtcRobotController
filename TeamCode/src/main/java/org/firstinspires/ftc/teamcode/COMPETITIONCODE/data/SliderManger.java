package org.firstinspires.ftc.teamcode.COMPETITIONCODE.data;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SliderManger {
    private DcMotor rotator; // Change the name
    private DcMotor controller;
    private double pos;
    private double pos2;

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
        if(run){
            controller.setPower(controllerPower/1.5);
            rotator.setPower(rotatorPower/1.75);
           pos = rotator.getCurrentPosition();
        }else{
            double currentPos = rotator.getCurrentPosition();
            if(pos - 10.0 >= currentPos || pos + 10.0 <= currentPos) {
                rotator.setPower(pos > currentPos ? 0.5 : -0.5);
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
