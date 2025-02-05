package org.firstinspires.ftc.teamcode.opmodes.test.auto;


// RR-specific imports
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;
public class SpecimenRedSpline extends LinearOpMode{
    public void runOpMode() {
        // I'm assuming you're at 0,0
        Pose2d initPose;
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(23.48, -69.05, Math.toRadians(90.00)));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(20, -60, 0))
                        .splineTo(new Vector2d(20, -40), Math.toRadians(0))
                        .splineTo(new Vector2d(60, -40), Math.toRadians(0))
                        .splineTo(new Vector2d(0, -40), Math.toRadians(0))
                        .splineTo(new Vector2d(0, -33), Math.toRadians(90))
                        .build());
    }

    //TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(23.48, -69.05, Math.toRadians(90.00)))
    //.lineTo(new Vector2d(48.62, -66.89))
    //.build();
}
