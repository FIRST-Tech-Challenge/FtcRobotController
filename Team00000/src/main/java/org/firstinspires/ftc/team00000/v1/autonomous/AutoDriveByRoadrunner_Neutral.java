package org.firstinspires.ftc.team00000.v1.autonomous;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import org.firstinspires.ftc.team00000.roadrunner.MecanumDrive;

        import java.lang.Math;

@Config
@Autonomous(name = "Neutral Samples (roadrunner)", group = "Autonomous")
public class AutoDriveByRoadrunner_Neutral extends LinearOpMode{

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9.5, -61.25, -Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-34)
                .setTangent(0)
                .lineToX(36)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(46, -9), 0)
                .setTangent(-Math.PI / 2)
                .lineToY(-50)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(56, -11), 0)
                .setTangent(-Math.PI / 2)
                .lineToY(-50)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(61, -13), 0)
                .setTangent(-Math.PI / 2)
                .lineToY(-50)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(36, -61), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(6.5, -34), Math.PI / 2)
                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(36, -61), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(4.5, -34), Math.PI / 2)
                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(36, -61), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(2.5, -34), Math.PI / 2)
                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(36, -61), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(0.5, -34), Math.PI / 2);

        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                        .build();

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
    }
}
