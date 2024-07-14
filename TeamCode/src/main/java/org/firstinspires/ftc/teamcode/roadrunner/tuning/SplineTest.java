package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.DrivetrainMecanumRR;
import org.firstinspires.ftc.teamcode.roadrunner.TankDriveRR;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (org.firstinspires.ftc.teamcode.tuning.TuningOpModes.DRIVE_CLASS.equals(DrivetrainMecanumRR.class)) {
            DrivetrainMecanumRR drive = new DrivetrainMecanumRR(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
        } else if (org.firstinspires.ftc.teamcode.tuning.TuningOpModes.DRIVE_CLASS.equals(TankDriveRR.class)) {
            TankDriveRR drive = new TankDriveRR(hardwareMap, beginPose);

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
