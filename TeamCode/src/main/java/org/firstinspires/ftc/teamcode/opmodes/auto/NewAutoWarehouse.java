package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoGrabber;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.opmodes.util.VisionToLiftHeight;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class NewAutoWarehouse extends LinearOpMode {
    public int multiplier = 1;
    public boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        TseDetector detector = new TseDetector(hardwareMap, "webcam", true, isRed);
        int height;
        double nextToWall = 70 - inchesToCoordinate(5.8D);

        MultipleTelemetry goodTelemetry = new MultipleTelemetry(telemetry);

        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoIntake intake = new AutoIntake(hardwareMap, eventThread);
        AutoGrabber grabber = new AutoGrabber(hardwareMap);
        AutoLift lift = new AutoLift(eventThread, hardwareMap, grabber);

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        final Pose2d initial = new Pose2d(0, multiplier * 70 - inchesToCoordinate(9),
                Math.toRadians(90 * multiplier));
        drive.setPoseEstimate(initial);

        final Pose2d liftPosition = new Pose2d(-2, 43.5 * multiplier,
                Math.toRadians(70 * multiplier));
        // Part 1: drive to alliance shipping hub
        final Trajectory part1;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(initial);
            builder.lineTo(new Vector2d(-3, 58 * multiplier));
            builder.lineToLinearHeading(liftPosition);
            part1 = builder.build();
        }

        // part 2: go to warehouse
        final Trajectory part2;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(liftPosition);
            builder.lineToLinearHeading(new Pose2d(0, (nextToWall + 1) * multiplier));
            builder.addDisplacementMarker(() -> drive.setWeightedDrivePower(new Pose2d(0, -0.2 * multiplier, 0)));
            builder.lineTo(new Vector2d(20, (nextToWall + 1) * multiplier));
            part2 = builder.build();
        }

        // where the robot **should** be after you intake
        final Pose2d intakeReturnPoint = new Pose2d(40, nextToWall * multiplier,
                Math.toRadians(0));
        // part 3: move back to Alliance Shipping hub. then you can go back to part 2 as needed.
        final Trajectory part3;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(intakeReturnPoint);
            builder.lineTo(new Vector2d(-3, nextToWall * multiplier));
            builder.lineToLinearHeading(liftPosition);
            part3 = builder.build();
        }

        final boolean[] liftUpdated = {false};
        Thread liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
                liftUpdated[0] = true;
            }
        });

        waitForStart();
        liftThread.start();
        height = detector.run();
        goodTelemetry.addData("height", height);
        goodTelemetry.update();

        drive.followTrajectoryAsync(part1);
        updateLoop(drive);
        if (!isStopRequested()) return;
        liftUpdated[0] = false;
        lift.setPosition(VisionToLiftHeight.getPosition(height));
        while (!liftUpdated[0] || lift.getState() != AutoLift.MovementStates.NONE) {
            if (isStopRequested()) {
                return;
            }
            drive.update();
        }

        drive.followTrajectoryAsync(part2);
        updateLoop(drive);
        if (isStopRequested()) return;



        drive.followTrajectoryAsync(part3);
        updateLoop(drive);
        if (isStopRequested()) return;

        drive.followTrajectoryAsync(part2);
        updateLoop(drive);
    }

    public void updateLoop(SampleMecanumDrive drive) {
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
        }
    }
}
