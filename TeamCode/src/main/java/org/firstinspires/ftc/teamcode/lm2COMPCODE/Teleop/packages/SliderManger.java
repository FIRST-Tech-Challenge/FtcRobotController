package org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.packages;

import com.qualcomm.robotcore.hardware.DcMotor;
/*
[/] Speedy mode
[/] Consistency
[] Simplify
*/
public class SliderManger {
    private DcMotor rotator; // Change the name
    private DcMotor controller;
    /*
    private double pos;
    private double pos2;
    */

    public void reset(){
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake motor
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset Encoder
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Init motor
    }

    public void init(DcMotor SlideController, DcMotor SlideRotator)
    {
        // Define Slides
        controller = SlideController;
        rotator = SlideRotator;
        controller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake motor
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Brake motor
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        /*
         pos = rotator.getCurrentPosition();  // Set pos 1
         pos2 = controller.getCurrentPosition(); // Set pos 2
        */
    }
    public void move(double controllerPower){
        controller.setPower(controllerPower); // Set power | Start moving
    }
    public void setPos(int pos, double power){ // Set position slowly
        rotator.setPower(power); // Start moving slowly
        rotator.setTargetPosition(pos); // Set target
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set mode. Why do I put this here and not earlier in the init.
    }
    /*
    public void setPos2(int pos){ // Set position quickly
        controller.setPower(1); // Start moving quickly
        controller.setTargetPosition(pos); // Set target
    }
    */
}
/*
NOTE!
    I removed the pos variables because they were not used.
* */