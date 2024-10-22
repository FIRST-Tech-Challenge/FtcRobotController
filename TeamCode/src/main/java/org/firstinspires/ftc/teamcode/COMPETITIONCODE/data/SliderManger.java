package org.firstinspires.ftc.teamcode.COMPETITIONCODE.data;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SliderManger {
    private DcMotor rotator; // Change the name
    private DcMotor controller;
    public void init(DcMotor SlideController, DcMotor SlideRotator)
    {
        controller = SlideController;
        rotator = SlideRotator;
    }
    public void move(double controllerPower, double rotatorPower){
        controller.setPower(controllerPower);
        rotator.setPower(rotatorPower);
    }
}
