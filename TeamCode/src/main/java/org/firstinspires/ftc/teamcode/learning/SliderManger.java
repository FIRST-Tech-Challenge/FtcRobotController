package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
