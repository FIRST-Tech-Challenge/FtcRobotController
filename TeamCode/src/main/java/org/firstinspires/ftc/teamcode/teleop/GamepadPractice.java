package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamepadPractice extends OpMode {



    @Override
    public void init(){


    }
    @Override
    public void loop(){

        double speedForward = gamepad1.left_stick_y / 2;
        double diffXJoyStick = gamepad1.left_stick_x-gamepad1.right_stick_x;
        double sumOfTriggers = gamepad1.left_trigger + gamepad1.right_trigger;
        telemetry.addData("left x",gamepad1.left_stick_x);
        telemetry.addData("left y",gamepad1.left_stick_y);
        telemetry.addData("right x",gamepad1.right_stick_x);
        telemetry.addData("right y",gamepad1.right_stick_y);
        telemetry.addData("a button:",gamepad1.a);
        telemetry.addData("b button:",gamepad1.b);
        telemetry.addData("difference x",diffXJoyStick);
        telemetry.addData("sumTriggers",sumOfTriggers);

    }
    //


}
