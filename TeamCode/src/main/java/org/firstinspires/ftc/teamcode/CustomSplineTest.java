package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class CustomSplineTest extends LinearOpMode {

    /*public class Relocalize implements Action {

    }*/

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(48, 48, Math.toRadians(270));
        MecanumDrive mecana = new MecanumDrive(hardwareMap, startPose);

        Action goToSub = mecana.actionBuilder(mecana.pose)
                //.splineTo(new Vector2d(3,2), Math.PI)
                .splineToSplineHeading(new Pose2d(36, 12, Math.PI), Math.PI)
                .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(40, 20, Math.toRadians(225)), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(48, 48, Math.toRadians(270)), Math.toRadians(90))
                .build();


        /*public Action relocalize{}*/

        /*Action goToBasket = mecana.actionBuilder(mecana.pose)
                //.splineTo(new Vector2d(3,2), Math.PI)
                .splineToSplineHeading(new Pose2d(48, 48, Math.toRadians(270)), Math.toRadians(90))
                .build();*/

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        goToSub
                )
        );
    }
}
