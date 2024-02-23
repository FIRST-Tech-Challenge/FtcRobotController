package org.firstinspires.ftc.teamcode.swift;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.shared.MotionHardware;


@TeleOp(name = "Drive Arm Test", group = "TeleOp2Driver")
public class TeleOpArmTestDS extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Left Trigger: ", gamepad1.left_trigger);
            telemetry.addData("Right Trigger: ", gamepad1.right_trigger);
            telemetry.update();


        }
        }

}



