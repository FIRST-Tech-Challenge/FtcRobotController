package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.SummerMec;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Disabled

@Autonomous(name= "RoadrunDemo", preselectTeleOp = "OneGPTeleop")
public class RoadrunDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SummerMecRobot robot = new SummerMecRobot(this, false);

        Pose2d startPose = new Pose2d(0, -48.5, 0);

        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence trajSeq = robot.roadrun.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(10, -58.5))
                .lineTo(new Vector2d(40, -58.5))
                .build();
        while (opModeIsActive()) {
            robot.followTrajectorySequenceAsync(trajSeq);
            robot.setFirstLoop(false);
            robot.roadrun.update();
        }
    }
}

