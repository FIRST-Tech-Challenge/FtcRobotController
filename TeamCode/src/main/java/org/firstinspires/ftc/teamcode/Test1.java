import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class Test1 extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-70, -35, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory myTrajectory = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-40, -32), Math.toRadians(35))
                .build();
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory.end())
                .splineTo(new Vector2d(-37, -16), Math.toRadians(90))
                .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(myTrajectory2);}
}