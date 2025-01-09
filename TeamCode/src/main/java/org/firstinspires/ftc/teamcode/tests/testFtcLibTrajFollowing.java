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
import java.util.List;


@Autonomous
@Config
public class testFtcLibTrajFollowing extends OpMode {
    PIDController txPID;
    PIDController tyPID;
    PIDController rotPID;
    FtcDashboard dash;
    Telemetry t2;
    Pose2d trajPose;
    Pose2d now;
    double rotPower;
    double xPower;
    double yPower;
    SwerveDrive drive;
    public static double tP = 1.2;
    public static double tI = 0.2;
    public static double tD = 0;
    public static double rP = 0.5;
    public static double rI = 0;
    public static double rD = 0;
    public static double P = 0.06;
    public static double I = 0.01;
    public static double D = 0.005;
    OptimalAngleCalculator angleCalculator;
    ElapsedTime trajTimer;
    List<Trajectory> trajSequence;
    Trajectory lastTrajThatWasFollowed;
    int trajTracker = 0;
    boolean stop = false;
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
        List<List<Pose2d>> pointlist = new ArrayList<>();
        pointlist.add(new ArrayList<>());
        // test turning first
        pointlist.get(0).add(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        pointlist.get(0).add(new Pose2d(new Translation2d(.5, 0), new Rotation2d(0)));
//        pointlist.get(0).add(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
//        pointlist.get(0).add(new Pose2d(new Translation2d(0.5, 0), new Rotation2d(0)));
//        pointlist.get(0).add(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
//        pointlist.get(0).add(new Pose2d(new Translation2d(0.5, 0), new Rotation2d(0)));
//        pointlist.get(0).add(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
//        pointlist.get(0).add(new Pose2d(new Translation2d(0.5, 0), new Rotation2d(0)));
//        pointlist.get(0).add(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
//        pointlist.get(0).add(new Pose2d(new Translation2d(0.5, 0), new Rotation2d(0)));

        // uncomment this once tuning rot PID is done
        // also probably tune tPID as well - often overshoots and then slowly goes back
        pointlist.add(new ArrayList<>());
        pointlist.get(1).add(new Pose2d(new Translation2d(0.5,0), new Rotation2d(Math.PI)));
        pointlist.get(1).add(new Pose2d(new Translation2d(0.5, 0.5), new Rotation2d(Math.PI)));


        trajSequence = new ArrayList<>();
        TrajectoryConfig traj1Config = new TrajectoryConfig(0.6, 0.3);

        for (List<Pose2d> points : pointlist) {
            trajSequence.add(
                TrajectoryGenerator.generateTrajectory(
                points,
                traj1Config
                )
            );
        }

        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
        txPID = new PIDController(tP, tI, tD);
        tyPID = new PIDController(tP, tI, tD);
        rotPID = new PIDController(rP, rI, rD);
        angleCalculator = new OptimalAngleCalculator();
        trajTimer = new ElapsedTime();
        now = drive.nowPose;
        lastTrajThatWasFollowed = trajSequence.get(0);
    }
    @Override
    public void init_loop() {
        drive.init_loop();
        telemetry.addData("last traj that was followed pose in", lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters);
        trajTimer.reset();
    }
    @Override
    public void loop() {
        Trajectory.State nowTraj;
        // the problem is here
        if (!stop) {
            nowTraj = getCurrentTraj().sample(trajTimer.seconds());
        } else {
            nowTraj = null;
            telemetry.addData("Reached,", trajTimer.seconds());
        }
        trajPose = nowTraj.poseMeters;
        now = drive.nowPose;
        rotPower = rotPID.calculate(now.getHeading(), trajPose.getHeading());
        xPower = txPID.calculate(now.getX(), trajPose.getX());
        yPower = tyPID.calculate(now.getY(), trajPose.getY());

        drive.loop(yPower, xPower, rotPower); // ignore the warning, is because of wpilib coord system

        doTelemetry(telemetry);
        doTelemetry(t2);
        txPID.setPID(tP, tI, tD);
        tyPID.setPID(tP, tI, tD);
        rotPID.setPID(rP, rI, rD);

    }
    public Trajectory getCurrentTraj() {
        telemetry.addData("Last State,", lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters);
        if ((Math.abs(now.getX() - lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters.getX()) < .05)
                && (Math.abs(now.getY() - lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters.getY())< .05)
                && (Math.abs(now.getHeading() - lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters.getHeading()) < 5 ||
                Math.abs(now.getHeading() + 360 - lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters.getHeading()) < 5 ||
                Math.abs(now.getHeading() - 360 - lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters.getHeading()) < 5)
                && (drive.timer.seconds() >= lastTrajThatWasFollowed.getTotalTimeSeconds())) {
////            // this returned true when...
////            /*
////            Traj Timer: 2.622744514
////            nowRot: 69.1708587249741
////            nowX: 0.15141628769543714
////            nowY: 0.22199981453085815
////
////            trajRot: 89.98545771535002
////            trajX: 0.4999907127707206
////            trajY: 0.4996454434762634
////
////            this is because eit wasn't comparing actual robot state. it was comparing
////            theoretical to theoretical.
////            need to check x, y, and heading.
////             */
            trajTimer.reset();
//            txPID.reset();
//            tyPID.reset();
//            rotPID.reset();
            if (trajTracker < trajSequence.size()) {
                trajTracker++;
                lastTrajThatWasFollowed = trajSequence.get(trajTracker);
            } else {
                stop = true;
            }
        }
        return lastTrajThatWasFollowed;
    }

    public void doTelemetry(Telemetry t) {
        t.addData("trajRot", trajPose.getRotation().getDegrees());
        t.addData("nowRot", now.getRotation().getDegrees());
        t.addData("trajX", trajPose.getX());
        t.addData("trajY", trajPose.getY());
        t.addData("nowX", now.getX());
        t.addData("nowY", now.getY());
        t.addData("powx", xPower);
        t.addData("powy", yPower);
        t.addData("powr", rotPower);
        t.addData("Traj Timer", trajTimer.seconds());
        t.addData("Current Traj", trajTracker);
        t.update();
    }
}
