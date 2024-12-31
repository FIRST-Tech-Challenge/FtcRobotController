package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Scanner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "servoTesting", group = "Teleops")
@Config
public class servoTesting extends OpMode {

    OpMode master;

    // outtake servos
    Servo outTakePivotRight;
    Servo outTakePivotLeft;
    Servo outTakeClaw;
    Servo outTakeFlip;
    // intake servos
    Servo intakePivotR;
    Servo intakePivotL;

    // x - intake rotator
    // y - outtake arm pivot
    // a - outtake flip
    // b - outtake claw

    // right bumper - increase value
    // left bumper - decrease value

    // NOTE: Upon setting to the servo/servos to be tuned, they go to position 0, wherever that may be

////////////////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void init() {
        // outtake servos
        outTakeClaw = hardwareMap.servo.get(("otc"));
        outTakeFlip = hardwareMap.servo.get(("otf"));
        outTakePivotRight = hardwareMap.servo.get("otpr");
        outTakePivotLeft = hardwareMap.servo.get("otpl");

        // intake servos
        intakePivotL = hardwareMap.servo.get(("itpl"));
        intakePivotR = hardwareMap.servo.get(("itpr"));
    }

    @Override
    public void loop() {
        master.telemetry.addLine("YA JASON WHICH TUNING METHOD IS BEING USED NOW BUD");
        if (master.gamepad2.x)
        {
            double rotatorConstant = 0;
            master.telemetry.addData("Servo position", rotatorConstant);
            master.telemetry.addData("Servo Being Tuned: ", "intake pivots");
            master.telemetry.update();
            if (master.gamepad2.right_bumper)
            {
                rotatorConstant += 0.1;
                intakePivotL.setPosition(rotatorConstant);
                intakePivotR.setPosition(rotatorConstant);
                master.telemetry.update();
            }
            if (master.gamepad2.left_bumper)
            {
                rotatorConstant -= 0.1;
                intakePivotL.setPosition(rotatorConstant);
                intakePivotR.setPosition(rotatorConstant);
                master.telemetry.addData("Servo position", rotatorConstant);
                master.telemetry.update();
            }
        }
        if (master.gamepad2.y)
        {
            double rotatorConstant = 0;
            master.telemetry.addData("Servo position", rotatorConstant);
            master.telemetry.addData("Servo Being Tuned: ", "outtake pivots");
            master.telemetry.update();
            if (master.gamepad2.right_bumper)
            {
                rotatorConstant += 0.1;
                outTakePivotLeft.setPosition(rotatorConstant);
                outTakePivotRight.setPosition(rotatorConstant);
                master.telemetry.update();
            }
            if (master.gamepad2.left_bumper)
            {
                rotatorConstant -= 0.1;
                outTakePivotLeft.setPosition(rotatorConstant);
                outTakePivotRight.setPosition(rotatorConstant);
                master.telemetry.update();
            }
        }
        if (master.gamepad2.a)
        {
            double rotatorConstant = 0;
            master.telemetry.addData("Servo position", rotatorConstant);
            master.telemetry.addData("Servo Being Tuned: ", "outtake flip");
            master.telemetry.update();
            if (master.gamepad2.right_bumper)
            {
                rotatorConstant += 0.1;
                outTakeFlip.setPosition(rotatorConstant);
                master.telemetry.update();
            }
            if (master.gamepad2.left_bumper)
            {
                rotatorConstant -= 0.1;
                outTakeFlip.setPosition(rotatorConstant);
                master.telemetry.update();
            }
        }
        if (master.gamepad2.b)
        {
            double rotatorConstant = 0;
            master.telemetry.addData("Servo position", rotatorConstant);
            master.telemetry.addData("Servo Being Tuned: ", "outtake claw grab");
            master.telemetry.update();
            if (master.gamepad2.right_bumper)
            {
                rotatorConstant += 0.1;
                outTakeClaw.setPosition(rotatorConstant);
                master.telemetry.update();
            }
            if (master.gamepad2.left_bumper)
            {
                rotatorConstant -= 0.1;
                outTakeClaw.setPosition(rotatorConstant);
                master.telemetry.update();
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void stop()
    {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
