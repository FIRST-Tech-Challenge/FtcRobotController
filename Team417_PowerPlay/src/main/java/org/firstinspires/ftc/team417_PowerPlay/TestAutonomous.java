package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team417_PowerPlay.drive.SampleMecanumDrive;

@Autonomous (name="strafe test")
public class TestAutonomous extends LinearOpMode {

    // Coordinates for various tiles referenced on page 46 of:
    // https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2-traditional.pdf
    // Robot's position starts at (0, 0) - all tile coordinates are in relation to this starting position
    private static final double Y_POSITION_A = -30;
    private static final double X_POSITION_C = 41;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traject1 = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(5, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, Y_POSITION_A), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(X_POSITION_C, Y_POSITION_A), Math.toRadians(0))
                .build();

        Trajectory traject2 = drive.trajectoryBuilder(traject1.end(), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(X_POSITION_C, Y_POSITION_A), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traject1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traject2);
    }
}
