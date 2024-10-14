package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class BlankTeleop extends LinearOpMode {

    // declare variables here
    //

    @Override
    public void runOpMode() {

        //    Do initialization things here



        while (opModeIsActive()) {
            // do op mode things here
            manageDriverControls();

        }
    }

    private void manageDriverControls()
    {
        if(gamepad1.triangle)
        {
            // do something if triangle is pushed
        }
        if(gamepad1.square)
        {
            // do something if square is pushed
        }

    }
}
