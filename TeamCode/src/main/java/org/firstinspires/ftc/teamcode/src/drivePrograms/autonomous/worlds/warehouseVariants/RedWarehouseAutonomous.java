package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.warehouseVariants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds.WorldsAutonomousProgram;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "🟥Red Warehouse Autonomous🟥", group = "RedWarehouse")
public class RedWarehouseAutonomous extends WorldsAutonomousProgram {
    final static Pose2d startPos = new Pose2d(7, -65, 0);
    final static Pose2d dropOffPos = new Pose2d(-12, -38, Math.toRadians(270));
    final static Pose2d whEntryPos = new Pose2d(20, -65, Math.toRadians(0));

    public RedWarehouseAutonomous() {
        super(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public TrajectorySequence toHub(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .strafeLeft(5)
                .lineToSplineHeading(dropOffPos.plus(new Pose2d(2, -3, 0)))
                //This path should not be replicated for additional freight since it is inefficient.
                .waitSeconds(1)
                .build();
    }

    public TrajectorySequence hubToWarehouse(SampleMecanumDrive drive, Pose2d startPos) {
        return drive.trajectorySequenceBuilder(startPos)
                .splineToSplineHeading(whEntryPos.plus(new Pose2d(0, 0, 0)), Math.toRadians(0))

                .forward(20)
                .build();
    }

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPos);

        final TrajectorySequence startToHub = toHub(drive, startPos);

        final TrajectorySequence hubToWH = hubToWarehouse(drive, startToHub.end());

        telemetry.addData("Setup", "Finished");
        telemetry.update();

        final BarcodePositions pos = monitorMarkerWhileWaitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(startToHub);

            this.dropOffItem(pos);

            drive.followTrajectorySequence(hubToWH);
            slide.setTargetLevel(HeightLevel.Down);
            slide.waitOn();

        }

    }
}
