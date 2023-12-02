package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.Robot;

@Autonomous(name="Autonomous Right Blue")
public class AutoRightBlue extends AutoBase {
    @Override
    public void runAutonomous(Robot robot, SampleMecanumDrive drive, TeamPropDetection.propLocation propLoc) {
        // hardware map to get motors and sensors
        TrajectorySequence dropPropPixelRight = drive.trajectorySequenceBuilder(c.preStartPoseBlueRight)
                //.lineTo(c.leftTeamProp)
                //.lineTo(c.centerTeamProp)
                .lineToLinearHeading(c.startPoseBlueRight)
                .lineToLinearHeading(c.rightTeamPropBlueRight)
                .back(3.5)
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(dropPropPixelRight.end())
                .lineToLinearHeading(c.leftBackdropIntermediateCenter)
                .lineToLinearHeading(c.leftBackdropLeft, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        robot.closeClaw = true;
        robot.updateSync();
        // Test propLoc here
        drive.followTrajectorySequence(dropPropPixelRight);

        robot.outtakePixels = true;
        robot.updateSync();
        drive.followTrajectorySequence(goToBackdrop);
        robot.closeClaw = false;
        robot.updateSync();
    }
}
