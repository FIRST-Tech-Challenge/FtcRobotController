package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TeleOp Drive", group="TeleOp")
public class DriveOp extends OpMode {

    OpTrain dt;


    @Override
    public void init() {
        telemetry.addData("Hello! Initializing!", "＼(⌒▽⌒)");
        telemetry.update();
        dt = new OpTrain(this, 10); //TODO
        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Get gamepad stuff

        float slowdownModifier = 1 - (gamepad1.right_trigger * 0.85f);

        float forwardDrive = gamepad1.right_stick_y * slowdownModifier;
        float panDrive = gamepad1.right_stick_x * slowdownModifier;
        float rotation = gamepad1.left_stick_x * slowdownModifier;

        //Manage panning

        dt.travel(forwardDrive, panDrive, rotation);

        //Telemetry

        telemetry.addData("Driving...", "");
        telemetry.addData("Forward Drive", forwardDrive);
        telemetry.addData("Pan Drive", panDrive);
        telemetry.addData("Rotation", rotation);
        telemetry.update();
    }
}
