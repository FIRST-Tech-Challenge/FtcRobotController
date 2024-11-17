package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "RoadRunner Autonomous", group = "Autonomous")
public class RoadRunnerAuto extends AutoBase {
    @Override
    public void runOpMode() {
        super.runOpMode(); // This runs the menu selection

        // Initialize RoadRunner drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, getStartingPose());

        // Wait for start
        waitForStart();

        if (isStopRequested())
            return;

        // Execute the trajectory based on selected position
        drive.actionBuilder(getStartingPose())
                .forward(24)
                .strafeRight(position.equals("Right") ? -36 : 36) // Strafe left if Right, right if Left
                .build();
    }

    private Pose2d getStartingPose() {
        // Get starting pose based on selected position
        double x = position.equals("Right") ? 36 : -36;
        double y = color.equals("Red") ? -60 : 60;
        double heading = color.equals("Red") ? Math.toRadians(90) : Math.toRadians(-90);

        return new Pose2d(x, y, heading);
    }
}