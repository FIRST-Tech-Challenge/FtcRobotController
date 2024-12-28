package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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
import org.firstinspires.ftc.teamcode.subsystems.swerve.OptimalAngleCalculator;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import kotlin.collections.ArrayDeque;

@Autonomous
@Config
public class testFtcLibTrajFollowing extends OpMode {
    Trajectory traj1;
    PIDController txPID;
    PIDController tyPID;
    PIDController rotPID;
    FtcDashboard dash;
    Telemetry t2;
    ElapsedTime timer;
    boolean eStop = false;
    Pose2d trajPose;
    Pose2d now;
    double xDiff;
    double yDiff;
    double rotDist;
    double rotPower;
    double xPower;
    double yPower;
    SwerveDrive drive;
    public static double tP = 1.2;
    public static double tI = 0.2;
    public static double tD = 0;
    public static double rP = 0;
    public static double rI = 0;
    public static double rD = 0;
    public static double P = 0.06;
    public static double I = 0.01;
    public static double D = 0.005;
    OptimalAngleCalculator angleCalculator;
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
                11, 11, 18, 18,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, P, I, D);
        List<Translation2d> tlist = new ArrayList<>();
//                Arrays.asList((new Translation2d(1,1)));
        TrajectoryConfig traj1Config = new TrajectoryConfig(0.6, 0.5);
        traj1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                tlist,
                new Pose2d(new Translation2d(.75,0), new Rotation2d(0)),
                traj1Config
                );
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
        txPID = new PIDController(tP, tI, tD);
        tyPID = new PIDController(tP, tI, tD);
        rotPID = new PIDController(rP, rI, rD);
        angleCalculator = new OptimalAngleCalculator();
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
        now = drive.nowPose;
        // TODO: for highlight: don't try to make this 0 because it goes negative like crazy
//        xDiff = trajPose.getX() - now.getX(); // don
//        yDiff = trajPose.getY() - now.getY();
        rotDist = (Math.toDegrees(now.getHeading()) - angleCalculator.calculateOptimalAngle(Math.toDegrees(now.getHeading()), Math.toDegrees(trajPose.getHeading())));
        rotPower = rotPID.calculate(rotDist, 0);
        xPower = txPID.calculate(now.getX(), trajPose.getX());
        yPower = tyPID.calculate(now.getY(), trajPose.getY());

        if (gamepad1.a) { eStop = true;}
        if (!eStop) { drive.loop(yPower, xPower, rotPower); }
        else {
            t2.addLine("Stopped");
            telemetry.addLine("Stopped");
        }
        doTelemetry(telemetry);
        doTelemetry(t2);
//        drive.getTelemetry(t2);
        txPID.setPID(tP, tI, tD);
        tyPID.setPID(tP, tI, tD);
        rotPID.setPID(rP, rI, rD);
    }

    public void doTelemetry(Telemetry t) {
        t.addData("trajRot", trajPose.getRotation().getDegrees());
        t.addData("nowRot", now.getRotation().getDegrees());
        t.addData("trajX", trajPose.getX());
        t.addData("trajY", trajPose.getY());
        t.addData("nowX", now.getX());
        t.addData("nowY", now.getY());
        t.addData("degDiff", rotDist);
        t.addData("powx", xPower);
        t.addData("powy", yPower);
        t.addData("powr", rotPower);
        t.addData("FL Angle", drive.states[0].angle.getDegrees());
        t.addData("FL Velocity", drive.states[0].speedMetersPerSecond);
        t.addData("FL Ticks", drive.driveMotors[0].getCurrentPosition());
        t.update();
    }
}
