package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;

//@TeleOp(name = "TacoTele")

public class TacoTele extends Taco_FF_Super_Class {

    @Override
    public void runOpMode() {

        initialization(false);
        waitForStart();
        if (opModeIsActive()) {
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            times_run++;
            while (opModeIsActive()) {
                drive();
                lift();
                intake();
                duck();
                time();
                telemetry.addData("Blue", blue);
                telemetry.addData("How many times we have run teleop", times_run);
                telemetry.update();
            }
        }
    }
}
