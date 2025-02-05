package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

@Autonomous(name = "RoadRunnerAutonomous")
@Disabled
public class RoadRunnerAutonomous extends LinearOpMode {
//    private Arm arm;
    private MecanumDrive drive;

    // Define arm positions using constants
    private static final double ARM_INIT_POSITION = 14.0; // Initial position
    private static final double ARM_LOW_POSITION = 12.0; // Low position
    private static final double ARM_HIGH_POSITION = 71.0; // High position
    private static final double ARM_MAX_POSITION = 95.0; // Maximum position

    @Override

    public void runOpMode() {
        // Initialize hardware
//        arm = new Arm(hardwareMap);
//        drive = new SampleMecanumDrive(hardwareMap);

        // Wait for the start signal
        waitForStart();

        // Define trajectories
//        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .forward(60) // Move forward 60 units
//                .build();
//
//        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
//                .turn(Math.toRadians(-90)) // Turn -90 degrees
//                .lineTo(new Vector2d(0, -36)) // Move to (0, -36)
//                .build();
//
//        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
//                .turn(Math.toRadians(90)) // Turn back to 0 degrees
//                .lineTo(new Vector2d(38, -36)) // Move to (38, -36)
//                .build();
//
//        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
//                .lineTo(new Vector2d(38, -10)) // Move to (38, -10)
//                .build();
//
//        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
//                .strafeTo(new Vector2d(44, -10)) // Strafe to (44, -10)
//                .build();
//
//        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
//                .turn(Math.toRadians(90)) // Turn to 90 degrees
//                .lineTo(new Vector2d(54, -10)) // Move to (54, -10)
//                .build();
//
//        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
//                .lineTo(new Vector2d(54, -58)) // Move to (54, -58)
//                .build();
//
//        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
//                .lineTo(new Vector2d(61, -58)) // Move to (61, -58)
//                .build();
//
//        // Execute trajectories
//        drive.followTrajectory(trajectory1);
//        arm.initializeArm(); // Initialize the arm
//        arm.setArmPosition(ARM_HIGH_POSITION); // Raise arm to high position
//        drive.followTrajectory(trajectory2);
//        drive.followTrajectory(trajectory3);
//        drive.followTrajectory(trajectory4);
//        drive.followTrajectory(trajectory5);
//        drive.followTrajectory(trajectory6);
//        drive.followTrajectory(trajectory7);
//        drive.followTrajectory(trajectory8);
//        arm.setArmPosition(ARM_INIT_POSITION); // Return arm to initial position
    }
}
