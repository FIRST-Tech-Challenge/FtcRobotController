package org.firstinspires.ftc.teamcode.tests;

import android.view.animation.LinearInterpolator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ClawSubsystem;

@Config
@TeleOp
public class ServoLimitsTesting extends LinearOpMode {
    ClawSubsystem claw;

    //Basket 0.01 - 0.48
    //Claw
    //Arm 0 - 0.925
    //FrontSl 0.05 -

    Telemetry dahsboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    public static double max = 1;
    public static double min = 0;

//    double last_min=min, last_max=max;
    double step = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new ClawSubsystem(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
//            if(gamepad1.dpad_up && pos < (1-step)) pos += step;
//
//            if(gamepad1.dpad_down && pos > step) pos -= step;

            if(gamepad1.dpad_up) claw.release();
            if(gamepad1.dpad_down) claw.grab();

            dahsboardTelemetry.addData("Smth", "");

//            servocr1.setPower(gamepad1.left_stick_y);
//            servocr2.setPower(-gamepad1.left_stick_y);
//            servo1.setPosition(pos);
//            servo2.setPosition(1-pos);
//            sleep(30);
        }
    }
}
