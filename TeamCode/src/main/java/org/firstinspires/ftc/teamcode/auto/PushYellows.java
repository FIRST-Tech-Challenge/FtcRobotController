package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

@TeleOp(name="Push Yellow", group="Linear OpMode")
public final class PushYellows extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, -55, Math.toRadians(90));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineToSplineHeading(new Pose2d(-57, -55, Math.toRadians(225)), Math.toRadians(225)) //basket position
                            .setTangent(Math.toRadians(225)).setReversed(true)
                            .splineToSplineHeading(new Pose2d(-40, -36, Math.toRadians(270)), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-40, -14), Math.toRadians(90))// beginning of loop back
                            .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(270)), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-54, -26), Math.toRadians(270)) //first sample pickup
                            .splineToSplineHeading(new Pose2d(-64, -46, Math.toRadians(225)), Math.toRadians(225)) //end of first lap

                            .setTangent(Math.toRadians(225)).setReversed(true)
                            .splineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(270)), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-48, -14), Math.toRadians(90))// beginning of loop back
                            .splineToSplineHeading(new Pose2d(-60, -14, Math.toRadians(270)), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-64, -26), Math.toRadians(270)) //second sample pickup
                            .splineToSplineHeading(new Pose2d(-64, -45, Math.toRadians(225)), Math.toRadians(225)) //end of second lap

                            .setTangent(Math.toRadians(225)).setReversed(true)
                            .splineToSplineHeading(new Pose2d(-52, -36, Math.toRadians(270)), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(90))// beginning of loop back
                            .splineToSplineHeading(new Pose2d(-68, -14, Math.toRadians(270)), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-72, -22), Math.toRadians(270)) //third sample pickup
                            .splineToConstantHeading(new Vector2d(-72, -45), Math.toRadians(270)) //end of third lap

                            .setTangent(Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-24, -6), Math.toRadians(0))
                            .build());


        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
