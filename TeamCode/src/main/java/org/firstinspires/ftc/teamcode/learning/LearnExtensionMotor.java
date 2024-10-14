package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class LearnExtensionMotor extends LinearOpMode {

    // declare variables here
    //

    @Override
    public void runOpMode() {

        //    Do initialization things here
        TouchSensor ExtensionLimit = hardwareMap.get(TouchSensor.class, "ExtensionLimit");
        DcMotor extensionMotor = hardwareMap.get(DcMotor.class,"Extension");
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // assuming forward is 'reaching'
        extensionMotor.setDirection(DcMotor.Direction.FORWARD);

        // using negative power to 'retract'
        extensionMotor.setPower(-0.1);
        while(!ExtensionLimit.isPressed()) {}
        extensionMotor.setPower(0);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
