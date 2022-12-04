package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team417_PowerPlay.drive.SampleMecanumDrive;
@Disabled
@Autonomous (name="Red right || Blue left ")
public class RedRightBlueLeftAuto extends LinearOpMode {

    // Coordinates for various tiles referenced on page 46 of:
    // https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2-traditional.pdf
    // Robot's position starts at (0, 0) - all tile coordinates are in relation to this starting position
    // X positions for rows, Y positions for different columns
    private static final double Y_POS_AWAY_FROM_WAll = 5;
    private static final double X_POS_6 = 36;
    private static final double Y_POS_E = 20;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traject1 = drive.trajectoryBuilder(new Pose2d())
                .forward(Y_POS_AWAY_FROM_WAll)
                .build();
        Trajectory traject2 = drive.trajectoryBuilder(traject1.end())
                .strafeLeft(X_POS_6)
                .build();

        Trajectory traject3 = drive.trajectoryBuilder(traject2.end())
                .forward(Y_POS_E)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traject1);
        drive.followTrajectory(traject2);
        drive.followTrajectory(traject3);
    }
}