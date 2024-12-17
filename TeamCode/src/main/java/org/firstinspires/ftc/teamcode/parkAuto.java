package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.RRLocalizationRead;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.RRLocalizationRead;

@Autonomous(name = "park auto", group = "autos")
public class parkAuto extends LinearOpMode {

    public RRLocalizationRead localizationRead;

    ElapsedTime totalTime = new ElapsedTime();

    DrivetrainControllers driveTrain;

    Mechanisms mechanisms;


    @Override
    public void runOpMode() {
        localizationRead = new RRLocalizationRead();
        localizationRead.initLocalization(hardwareMap);

        driveTrain = new DrivetrainControllers();
        driveTrain.init(this);

        mechanisms = new Mechanisms();


        waitForStart();

        if (opModeIsActive()) {

            driveTrain.moveForwardByInches(60.0, 10);
            //driveTrain.moveForwardByInches(-60, 10);// Moves the robot forward by 10 inches

        }
    }


}