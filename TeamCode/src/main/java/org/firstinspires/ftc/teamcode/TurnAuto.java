package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Turn Auto", group = "Tests")
public class TurnAuto extends LinearOpMode {
    protected MecanumDrive drive;

    protected Action turn;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        turn = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(90))
                .build();

        waitForStart();

        Actions.runBlocking(turn);
    }
}
