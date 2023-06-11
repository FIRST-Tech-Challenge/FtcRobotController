package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class HomeworkChapter46Ethan extends OpMode{

    @Override
    public void init() {

    }
    @Override
    public  void loop() {
        double LeftY = gamepad1.left_stick_y;
        double LeftX = gamepad1.left_stick_x;
        double RightY = gamepad1.right_stick_y;
        double RightX = gamepad1.right_stick_x;

        //problem 1 - turbo button
        if(gamepad1.a){
            telemetry.addData("Foward Speed", "");
        }
        else{
            LeftY *= 0.5;
            LeftX *= 0.5;
            RightY *= 0.5;
            RightX *= 0.5;
        }


        //problem 2 - crazy mode

        if(gamepad1.a){
            double temp = LeftX;
            LeftX = LeftY;
            LeftY = temp;
            temp = RightX;
            RightX = RightY;
            RightY = temp;
        }
        else{
            telemetry.addData("Joystick is normal", "");
        }


    }
}

