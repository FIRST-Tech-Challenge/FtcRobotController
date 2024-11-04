package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name="Auto Tester")
public class AutoTesting extends LinearOpMode {

    private Pose2d startPosition = new Pose2d(-12,-64,Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive roadRunner = new MecanumDrive(hardwareMap,startPosition);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                roadRunner.actionBuilder(startPosition)
                        .strafeTo( new Vector2d(-12,-40))
                        .build()
        ));
    }
}
