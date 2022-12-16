package org.firstinspires.ftc.teamcode.roadrunner.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(group="drive")
public class LineToLinearHeading extends LinearOpMode {
    TurtleRobotAuto robot = new TurtleRobotAuto(this);

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    long slide;


    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
<<<<<<< Updated upstream
//                .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)))
                .forward(25)
//                .strafeLeft(15)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(45)
                .build();
=======
                .forward(40)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(-45)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(20)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(0, -80, Math.toRadians(135)))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(45)))
                .build();



>>>>>>> Stashed changes
        waitForStart();

        if(isStopRequested()) return;
        slide = 1700;
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        LinearSlide(0.75, 2000);
        LinearSlide(0, 0);
        robot.ArmServo.setPower(-1);
        sleep(100);
        robot.ArmServo.setPower(0);
        drive.followTrajectory(traj3);
        LinearSlide(-0.75, 1700);
        LinearSlide(0, 0);
        LinearSlide(-0.75, 1700);
        LinearSlide(0, 0);
        while (slide != 1100) {
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj1);
            robot.ArmServo.setPower(1);
            sleep(100);
            robot.ArmServo.setPower(0);
            drive.followTrajectory(traj5);
            LinearSlide(0.75, slide);
            robot.ArmServo.setPower(-1);
            sleep(100);
            robot.ArmServo.setPower(0);
            drive.followTrajectory(traj3);
            LinearSlide(-0.75, slide);
            LinearSlide(0, 0);
            slide -= 300;
        }





    }
    public void LinearSlide(double speed, long time){
        robot.leftslidemotor.setPower(speed);
        robot.rightslidemotor.setPower(speed);
        sleep(time);

<<<<<<< Updated upstream
//        drive.followTrajectory(traj1);
        drive.followTrajectory(traj1);
=======
>>>>>>> Stashed changes
    }
}






