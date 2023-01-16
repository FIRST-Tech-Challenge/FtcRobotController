package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "FSM state right")
public class FsmStateRightSide extends BaseAutonomous{

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto();


        Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        Trajectory traject1 = drive.trajectoryBuilder(startPose, false)
                .splineToConstantHeading(new Vector2d(-36, 10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36, 45), Math.toRadians(-90))
                .build();

        Trajectory traject2 = drive.trajectoryBuilder(traject1.end(), false)
                .splineToConstantHeading(new Vector2d(-24, 36), Math.toRadians(-90))
                .build();
        // score on mid junction

        Trajectory traject3 = drive.trajectoryBuilder(traject2.end(),false)
                .splineToConstantHeading(new Vector2d(-36,20), Math.toRadians(-90))
                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                .build();

        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .splineTo(new Vector2d(-36, 20), Math.toRadians(45))
                .build();

        //Parking Cases


        telemetry.addLine("ready for start");
        telemetry.update();
        waitForStart();
        //telemetry.addLine("started");
        //telemetry.update();
        //sleep(3000);

        drive.followTrajectory(traject1);
        drive.followTrajectory(traject2);
        drive.followTrajectory(traject3);
        drive.followTrajectory(traject4);
    }
}
