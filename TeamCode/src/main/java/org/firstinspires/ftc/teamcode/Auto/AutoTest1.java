package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Visionportal3;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/* velocity + acceleration limit command in trajectory movements:
SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
 */

@Autonomous(name = "AutoTest1")
public class AutoTest1 extends LinearOpMode {


    // start pose with random estimated values; will change later
    // we can also make 4 different pose2d values for each of the start positions so changing at start will be easier
    Pose2d blueStart1 = new Pose2d(-37, 72, Math.toRadians(-90));
    Pose2d blueStart2 = new Pose2d(37, 72, Math.toRadians(-90));
    Pose2d redStart1 = new Pose2d(-37, -72, Math.toRadians(90));
    Pose2d redStart2 = new Pose2d(37, -72, Math.toRadians(90));

    Vector2d blue1LeftMark = new Vector2d(-48, 34);
    Vector2d blue1CenterMark = new Vector2d(-35, 27);
    Vector2d blue1RightMark = new Vector2d(-25, 34);

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(blueStart1);


        // move robot to pixel stash (will change values later)
        TrajectorySequence test = drive.trajectorySequenceBuilder(blueStart1)
                // detect y-axis custom object from starting location
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(blue1RightMark)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-35,57, Math.toRadians(0))) // start moving towards backboard for yellow pixel
                .lineToConstantHeading(new Vector2d(10, 57))
                .splineToConstantHeading(new Vector2d(52, 36), Math.toRadians(0))
                .waitSeconds(0.75) // drop yellow pixel

                .splineToConstantHeading(new Vector2d(10, 57), Math.toRadians(0)) // go back for two white pixels
                .lineToConstantHeading(new Vector2d(-35, 57))
                .splineToConstantHeading(new Vector2d(-65, 36), Math.toRadians(0))
                .waitSeconds(0.75) // intake two white pixels
                .splineToConstantHeading(new Vector2d(-35, 57), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(10, 57))
                .splineToConstantHeading(new Vector2d(52, 36), Math.toRadians(0))
                .waitSeconds(0.75) // drop two white pixels on back board
                .lineTo(new Vector2d(56, 15)) // park in a spot that the other team probably won't park
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(test);
    }
}
