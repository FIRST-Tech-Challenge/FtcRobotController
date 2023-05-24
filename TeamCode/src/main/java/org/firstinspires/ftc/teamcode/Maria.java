import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class Maria extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-70, -35, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory myTrajectory = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-15, 13), Math.toRadians(0))
                .build();

        Trajectory myTrajectory1 = drive.trajectoryBuilder(myTrajectory.end())
                .back(52)
                .build();
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory1.end())
                .lineToConstantHeading(new Vector2d(-15, -35))
                .build();
        Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory2.end())
                .back(52)
                .build();

        waitForStart();


        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(myTrajectory1);
        drive.followTrajectory(myTrajectory2);
        drive.followTrajectory(myTrajectory3);
    }
}