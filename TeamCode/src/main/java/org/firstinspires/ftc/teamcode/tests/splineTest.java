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


@Autonomous
@Config
public class splineTest extends OpMode {
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
    ElapsedTime velocityTimer;
    SwerveDrive drive;
    public static double tP = .22;
    public static double tI = 0;
    public static double tD = 0.0001;
    public static double rP = .075;
    public static double rI = 0;
    public static double rD = 0;
    public static double kV = 0.05; // Velocity constant
    public static double kA = 0; // Acceleration constant
    public static double kStatic = 0.0; // Static constant
    double previousPosX;
    double previousPosY;
    double prevVelX;
    double prevVelY;
    double actualVelX;
    double actualVelY;
    double prevRobotX;
    double prevRobotY;
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
                encoderNames, driveNames, angleNames,0, 0, 1);

        builder = new TrajectorySequenceBuilder(new Pose2d(0, 0, 1+Math.PI/2),
                drive.velocityConstraint, drive.accelerationConstraint,
                drive.maxAngVel, drive.maxAngAccel); // TODO: Maybe bad radians/degrees
        trajSequence = builder
                .forward(72)
                .back(72)
                .forward(72)
                .back(72)
                .forward(72)
                .back(72)
                .forward(72)
                .back(72)
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
        velocityTimer = new ElapsedTime();
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
        prevVelX = 0;
        prevVelY = 0;
        previousPosX = trajSequence.start().getX();
        previousPosY = trajSequence.start().getY();
        prevRobotX = now.getX();
        prevRobotY = now.getY();
    }
    //    public Pose2d rotationMatrix(Pose2d robotPose) {
//
//    }

    @Override
    public void loop() {
        // Get the target pose at the current time
        trajPose = getPoseAtTime(trajSequence, trajTimer.seconds());
        // Get the current pose of the robot
        now = rotateFTCLibPose(drive.nowPose);
        // V and A from profile
        double desiredVelocityX = (trajPose.getX() - previousPosX) / velocityTimer.seconds();
        double desiredVelocityY = (trajPose.getY() - previousPosY) / velocityTimer.seconds();
        double desiredAccelerationX = (desiredVelocityX - prevVelX) / velocityTimer.seconds();
        double desiredAccelerationY = (desiredVelocityY - prevVelY) / velocityTimer.seconds();

        actualVelX = (now.getX() - prevRobotX) / velocityTimer.seconds();
        actualVelY = (now.getY() - prevRobotY) / velocityTimer.seconds();

        // Calculate feedforward terms
        double feedforwardX = kV * desiredVelocityX + kA * desiredAccelerationX + kStatic;
        double feedforwardY = kV * desiredVelocityY + kA * desiredAccelerationY + kStatic;

        // Calculate PID outputs
        rotPower = -rotPID.calculate(now.getHeading(), trajPose.getHeading());
        xPower = txPID.calculate(now.getX(), trajPose.getX()) + feedforwardX;
        yPower = -tyPID.calculate(now.getY(), trajPose.getY()) + feedforwardY;

        // Apply the calculated powers to the robot's drive system
        drive.loopFC(now.getHeading(), xPower, yPower, rotPower);

        // Update telemetry
        doTelemetry(telemetry);
        doTelemetry(t2);

        // Update PID coefficients
        txPID.setPID(tP, tI, tD);
        tyPID.setPID(tP, tI, tD);
        rotPID.setPID(rP, rI, rD);

        // Update previous velocities
        previousPosY = trajPose.getY();
        previousPosX = trajPose.getX();
        prevVelX = desiredVelocityX;
        prevVelY = desiredVelocityY;
        velocityTimer.reset();
    }
    public void doTelemetry(Telemetry t) {
        t.addData("targ Vel X", prevVelX);
        t.addData("targ Vel Y", prevVelY);
        t.addData("Actual Vel X", actualVelX);
        t.addData("Actual Vel Y", actualVelY);


//        drive.getTelemetry(t);
//        t.addData("trajRot", trajPose.getHeading());
//        t.addData("nowRot", now.getHeading());
//        t.addData("trajX", trajPose.getX());
//        t.addData("trajY", trajPose.getY());
//        t.addData("nowX", now.getX());
//        t.addData("nowY", now.getY());
//        t.addData("powx", xPower);
//        t.addData("powy", yPower);
//        t.addData("powr", rotPower);
//        t.addData("Traj Timer", trajTimer.seconds());
//        t.addData("Current Traj", trajTracker);
        t.update();
    }
}
