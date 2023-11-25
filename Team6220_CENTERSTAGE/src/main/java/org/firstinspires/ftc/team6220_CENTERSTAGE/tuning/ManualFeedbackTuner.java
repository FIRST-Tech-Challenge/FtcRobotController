package org.firstinspires.ftc.team6220_CENTERSTAGE.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, -48, Math.toRadians(0)));

            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(24, -48, Math.toRadians(0)))
                            .setTangent(Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(24,0,Math.toRadians(0)), Math.toRadians(90))
                            .endTrajectory()
                            .splineToSplineHeading(new Pose2d(24,-48,Math.toRadians(0)), Math.toRadians(90))
                            .build());
            }
        } else {
            throw new AssertionError();
        }
    }
}
