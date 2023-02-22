package org.firstinspires.ftc.masters.oldAndUselessStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

//@Autonomous(name = "Judging LEDs", group = "test")
public class JudgingLights extends LinearOpMode {

    public DigitalChannel redLED, greenLED, redLED2, greenLED2;

    @Override
    public void runOpMode() throws InterruptedException {

        redLED = (DigitalChannel) hardwareMap.get("red");
        greenLED = (DigitalChannel) hardwareMap.get("green");
        redLED2 = (DigitalChannel) hardwareMap.get("red2");
        greenLED2 = (DigitalChannel) hardwareMap.get("green2");

        waitForStart();


        while (opModeIsActive()) {
            redLED.setMode(DigitalChannel.Mode.OUTPUT);
            greenLED.setMode(DigitalChannel.Mode.OUTPUT);
            redLED2.setMode(DigitalChannel.Mode.OUTPUT);
            greenLED2.setMode(DigitalChannel.Mode.OUTPUT);

            redLED.setState(false);
            greenLED2.setState(false);
            redLED2.setState(true);
            greenLED.setState(true);
        }
    }


}

