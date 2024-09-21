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
                            .splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(225)), Math.toRadians(225))
                            .setTangent(Math.toRadians(225))
                            .lineToY(-56) //basket location
                            .strafeToSplineHeading(new Vector2d(-35, -35), Math.toRadians(270))
                            .setTangent(Math.toRadians(90))
                            .lineToY(-3)
                            .setTangent(Math.toRadians(180))
                            .lineToX(-46)  //first sample pickup
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)), Math.toRadians(225)) //end of 1st lap

                            .setTangent(Math.toRadians(45))
                            .splineToLinearHeading(new Pose2d(-45, -3, Math.toRadians(270)), Math.toRadians(90))
                            .setTangent(Math.toRadians(180))
                            .lineToX(-55)  //second sample pickup
                            .setTangent(Math.toRadians(270))
//              .strafeToSplineHeading(new Vector2d(-50, -50),Math.toRadians(225)) //these three lines are alternate backup path
//              .setTangent(Math.toRadians(225))
//              .lineToY(-56) //end of 2nd lap
                            .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)), Math.toRadians(225)) //end of 2nd lap

                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-54, -3, Math.toRadians(270)), Math.toRadians(90))
                            .setTangent(Math.toRadians(180))
                            .lineToX(-61)
                            .setTangent(Math.toRadians(270))
                            .lineToY(-50) //end of 3rd lap

                            .setTangent(Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(-24, -10), Math.toRadians(0))
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
