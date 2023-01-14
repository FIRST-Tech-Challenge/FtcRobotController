package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        Servo gr = hardwareMap.servo.get("grabber");

        double p = 0;
        double v = 1;


        waitForStart();

        while (opModeIsActive()){
            gr.setPosition(p - gamepad1.left_stick_y * v);

            if (A_pressed()){
                p = p - gamepad1.left_stick_y * v;
                v /= 2;
            }
            telemetry.addData("postion", p - gamepad1.left_stick_y * v);
            telemetry.update();
        }

    }

    private boolean a = true;
    public boolean A_pressed(){
        if (gamepad1.a){
            if(a){
                a = false;
                return true;
            }
        } else a = true;
        return false;
    }
}