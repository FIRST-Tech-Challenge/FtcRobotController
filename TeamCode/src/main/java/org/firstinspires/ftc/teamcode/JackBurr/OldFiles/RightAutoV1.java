package org.firstinspires.ftc.teamcode.JackBurr.OldFiles;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RightAutoV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Pick SampleMecanumDrive for dashboard and RRMecanumDrive for no dashboard
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Pose2d startPose = new Pose2d(60, 0, Math.toRadians(0));
        //drive.setPoseEstimate(startPose);

        //Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.strafeTo(new Vector2d(30, 0))
                //.build();

        //Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //.strafeTo(new Vector2d(60, 64))
                //.build();


        waitForStart();
        if (isStopRequested()) return;
        //drive.followTrajectory(traj1);
        //drive.followTrajectory(traj2);
    }
}
