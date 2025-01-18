package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.equation.Sequence;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.swerve.OptimalAngleCalculator;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

import java.util.List;
import java.util.Vector;


@Autonomous
@Config
public class firstAuto extends OpMode {
    PIDController txPID;
    PIDController tyPID;
    PIDController rotPID;
    FtcDashboard dash;
    Pose2d now;
    Pose2d trajPose;
    TrajectorySequenceBuilder builder;
    Telemetry t2;
    double rotPower;
    double xPower;
    double yPower;
    TrajectorySequence trajSequence;
    SwerveDrive drive;
    public static double tP = .2;
    public static double tI = 0;
    public static double tD = 0;
    public static double rP = .2;
    public static double rI = 0;
    public static double rD = 0;
    public static double P = 0.06;
    public static double I = 0.01;
    public static double D = 0.005;
    OptimalAngleCalculator angleCalculator;
    ElapsedTime trajTimer;
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
    public Pose2d rotateFTCLibPose(com.arcrobotics.ftclib.geometry.Pose2d odoPose) {
        Pose2d tempPose = new Pose2d(odoPose.getY()*-1,odoPose.getX(), odoPose.getHeading()+Math.PI/2);
        return tempPose;
    }
    @Override
    public void init() {
        drive = new SwerveDrive(
                11, 11, 18, 18,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, -66, -12, 0);

        builder = new TrajectorySequenceBuilder(new Pose2d(12, -66, Math.PI/2),
                drive.velocityConstraint, drive.accelerationConstraint,
                drive.maxAngVel, drive.maxAngAccel); // TODO: Maybe bad radians/degrees
        trajSequence = builder
                .forward(10)
                .strafeRight(9)
                .splineTo(new Vector2d(44,-11), 0)
                .lineToConstantHeading(new Vector2d(48, -56))
                .lineToConstantHeading(new Vector2d(50, -11))
                .splineToConstantHeading(new Vector2d(52, -56), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(60, -11))
                .splineToConstantHeading(new Vector2d(63, -56), Math.toRadians(90))
                .build();


        // takes the robot's location and velocity and the target location and returns the power to get there



        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
        txPID = new PIDController(tP, tI, tD);
        tyPID = new PIDController(tP, tI, tD);
        rotPID = new PIDController(rP, rI, rD);
        angleCalculator = new OptimalAngleCalculator();
        trajTimer = new ElapsedTime();
        now = rotateFTCLibPose(drive.nowPose);
    }
    public Pose2d getPoseAtTime(TrajectorySequence trajectorySequence, double time) {
        double accumulatedTime = 0.0;
        for (int i = 0; i < trajectorySequence.size(); i++) {
            TrajectorySegment segment = (TrajectorySegment) trajectorySequence.get(i);
            double segmentDuration = segment.getDuration();
            if (accumulatedTime + segmentDuration >= time) {
                double timeInSegment = time - accumulatedTime;
                return segment.getPoseAtTime(timeInSegment);
            }
            accumulatedTime += segmentDuration;
        }
        // If the time exceeds the total duration, return the end pose
        return trajectorySequence.end();
    }
    @Override
    public void init_loop() {
        drive.init_loop();

//        telemetry.addData("last traj that was followed pose in", lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters);
        trajTimer.reset();
    }
    @Override
    public void loop() {
        // the problem is here
        trajPose = getPoseAtTime(trajSequence, trajTimer.seconds());
        now = rotateFTCLibPose(drive.nowPose);
        rotPower = -rotPID.calculate(now.getHeading(), trajPose.getHeading());
        xPower = txPID.calculate(now.getX(), trajPose.getX());
        yPower = -tyPID.calculate(now.getY(), trajPose.getY());

        drive.loop(xPower, yPower, rotPower); // ignore the warning, is because of wpilib coord system

        doTelemetry(telemetry);
        doTelemetry(t2);
        txPID.setPID(tP, tI, tD);
        tyPID.setPID(tP, tI, tD);
        rotPID.setPID(rP, rI, rD);

    }
    public void doTelemetry(Telemetry t) {
        t.addData("trajRot", trajPose.getHeading());
        t.addData("nowRot", now.getHeading());
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
