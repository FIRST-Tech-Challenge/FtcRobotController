package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.tools.PoseStorage;

@Autonomous(name="world_park")
public class Park extends LinearOpMode
{

    @Override
    public void runOpMode()
    {


        telemetry.setMsTransmissionInterval(50);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectoryRight = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(25)
                .back(25)
                .build();





        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {

        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        // WRITE CODE FOR AUTONOMOUS
        drive.followTrajectorySequence(trajectoryRight);
        telemetry.update();
        PoseStorage.currentPose = drive.getPoseEstimate();

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

}