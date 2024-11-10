package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class SampleAuto01 extends OpMode {
    SampleMecanumDrive drive;
    Pose2d start_pos = new Pose2d(0,0,0);
    Pose2d current_position = start_pos;
    Pose2d end = null;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void start(){
        drive.setPoseEstimate(start_pos);
        current_position = moveForward(60);
        current_position = turnLeft(90);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(){
        current_position = turnRight(90);
        current_position = moveBackward(60);
    }

    public Pose2d moveForward(double distance){
        Trajectory trajectory;
        if(end == null) {
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(distance)
                    .build();
        }
        else {
            trajectory = drive.trajectoryBuilder(end)
                    .forward(distance)
                    .build();
        }
        drive.followTrajectory(trajectory);
        drive.updatePoseEstimate();
        end = trajectory.end();
        return trajectory.end();
    }

    public Pose2d moveBackward(double distance){
        Trajectory trajectory;
        if(end == null) {
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(distance)
                    .build();
        }
        else {
            trajectory = drive.trajectoryBuilder(end)
                    .back(distance)
                    .build();
        }
        drive.followTrajectory(trajectory);
        drive.updatePoseEstimate();
        end = trajectory.end();
        return trajectory.end();
    }

    public Pose2d strafeLeft(double distance){
        Trajectory trajectory;
        if(end == null) {
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(distance)
                    .build();
        }
        else {
            trajectory = drive.trajectoryBuilder(end)
                    .strafeLeft(distance)
                    .build();
        }
        drive.followTrajectory(trajectory);
        drive.updatePoseEstimate();
        end = trajectory.end();
        return trajectory.end();
    }

    public Pose2d strafeRight(double distance){
        Trajectory trajectory;
        if(end == null) {
            trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight(distance)
                    .build();
        }
        else {
            trajectory = drive.trajectoryBuilder(end)
                    .strafeRight(distance)
                    .build();
        }
        drive.followTrajectory(trajectory);
        drive.updatePoseEstimate();
        end = trajectory.end();
        return trajectory.end();
    }

    public Pose2d turnLeft(double degrees){
        TrajectorySequence trajectory;
        if(end == null) {
            trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(degrees))
                    .build();
        }
        else {
            trajectory = drive.trajectorySequenceBuilder(end)
                    .turn(Math.toRadians(degrees))
                    .build();
        }
        drive.followTrajectorySequence(trajectory);
        drive.updatePoseEstimate();
        end = trajectory.end();
        return trajectory.end();
    }

    public Pose2d turnRight(double degrees){
        TrajectorySequence trajectory;
        if(end == null) {
            trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(-Math.toRadians(degrees))
                    .build();
        }
        else {
            trajectory = drive.trajectorySequenceBuilder(end)
                    .turn(-Math.toRadians(degrees))
                    .build();
        }
        drive.followTrajectorySequence(trajectory);

        end = trajectory.end();
        return trajectory.end();
    }

}
