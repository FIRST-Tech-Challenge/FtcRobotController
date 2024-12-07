package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;

import java.util.Arrays;
import java.util.List;

@Autonomous
public class testFtcLibTrajFollowing extends OpMode {
    Trajectory traj1;
    FtcDashboard dash;
    Telemetry t2;
    ElapsedTime timer;
    boolean eStop = false;
    Pose2d trajPose;
    Pose2d now;
    double xDiff;
    double yDiff;
    double degDiff;
    double rotPower;
    SwerveDrive drive;
    public static double P = 0.06;
    public static double I = 0.01;
    public static double D = 0.005;
    public static double dP = 1e-5;
    public static double dI = 1e-5;
    public static double dD = 1e-5;
    String[] encoderNames = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    String[] driveNames = {
            "fl_drive",
            "fr_drive",
            "bl_drive",
            "br_drive"
    };
    String[] angleNames = {
            "fl_angle",
            "fr_angle",
            "bl_angle",
            "br_angle"
    };
    @Override
    public void init() {
        drive = new SwerveDrive(
                18, 18, 12, 12,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, P, I, D, dP, dI, dD);
        List<Translation2d> tlist = Arrays.asList((new Translation2d(1,1)));
        TrajectoryConfig traj1Config = new TrajectoryConfig(0.5, 0.5);
        traj1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(0, 0), new Rotation2d(90)),
                tlist,
                new Pose2d(new Translation2d(2,2), new Rotation2d(0)),
                traj1Config
                );
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }
    @Override
    public void init_loop() {
        drive.init_loop();
        telemetry.addLine("Press A to stop trajectory following");
    }
    @Override
    public void loop() {
        Trajectory.State nowTraj = traj1.sample(drive.timer.seconds());
        trajPose = nowTraj.poseMeters;
        now = drive.odo.getPoseMeters();
        xDiff = trajPose.getX() - now.getX();
        yDiff = trajPose.getY() - now.getY();
        degDiff = trajPose.getRotation().getDegrees() - now.getRotation().getDegrees();
        rotPower = Math.signum(degDiff) * degDiff;
        if (gamepad1.a) { eStop = true;}
        if (!eStop) { drive.loop(xDiff/100000000, yDiff/1000000,rotPower/2); }
        else {
            t2.addLine("Stopped");
            telemetry.addLine("Stopped");
        }
        doTelemetry(telemetry);
        doTelemetry(t2);
    }

    public void doTelemetry(Telemetry t) {
        t.addData("trajPose", trajPose);
        t.addData("nowPose", now);
        t.addData("xDiff", xDiff);
        t.addData("yDiff", yDiff);
        t.addData("degDiff", degDiff);
        t.update();
    }
}
