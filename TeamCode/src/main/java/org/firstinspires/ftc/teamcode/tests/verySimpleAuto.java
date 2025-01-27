package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.nematocyst;
import org.firstinspires.ftc.teamcode.subsystems.swerve.OptimalAngleCalculator;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

@Autonomous
public class verySimpleAuto extends OpMode {
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
    TrajectorySequence trajSequence1;
    SwerveDrive drive;
    public static double tP = .22;
    public static double tI = 0;
    public static double tD = 0.0001;
    public static double rP = .075;
    public static double rI = 0;
    public static double rD = 0;
    public static double P = 0.03;
    public static double I = 0.01;
    public static double D = 0.005;
    nematocyst n;
    OptimalAngleCalculator angleCalculator;
    ElapsedTime trajTimer;
    int trajTracker = 0;
    public enum STATES {
        INIT,
        ARM_UP,
        FORWARD,
        SCORE_SPECIMEN,
        RELEASE_AND_PARK,
        STOP;
    }
    STATES previousState = STATES.INIT;
    STATES currentState = STATES.INIT;
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
    public void init() {
        n = new nematocyst(this);
        n.init("pivot", "slide", "wrist", "claw");
        drive = new SwerveDrive(
                11, 11, 18, 18,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, -66, -12, 0);

        builder = new TrajectorySequenceBuilder(new Pose2d(12, -66, Math.PI/2),
                drive.velocityConstraint, drive.accelerationConstraint,
                drive.maxAngVel, drive.maxAngAccel); // TODO: Maybe bad radians/degrees
        trajSequence = builder
                .forward(12)
                .build();
        trajSequence1 = builder
                .back(10)
                .strafeRight(42)
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
        n.grab();
        n.wristIn();
    }
    @Override
    public void init_loop() {
        drive.init_loop();
//        telemetry.addData("last traj that was followed pose in", lastTrajThatWasFollowed.getStates().get(lastTrajThatWasFollowed.getStates().size()-1).poseMeters);
        trajTimer.reset();
    }
    @Override
    public void start() {
        currentState = STATES.ARM_UP;
    }
    @Override
    public void loop() {
        switch(currentState) {
            case INIT:
                break;
            case ARM_UP:
                if (previousState != currentState) {
                    n.goSpecimen(2);
                    n.wristSpecimen();
                    previousState = STATES.ARM_UP;
                } else if (n.isAtTargetHeight()) {
                    currentState = STATES.FORWARD;
                }
                n.loop();
                break;
            case FORWARD:
                if (previousState != currentState) {
                    trajTimer.reset();
                    previousState = STATES.FORWARD;
                } else if (checkTrajEndReached(trajSequence)) {
                    currentState = STATES.SCORE_SPECIMEN;
                }
                trajPose = getPoseAtTime(trajSequence, trajTimer.seconds());
                now = rotateFTCLibPose(drive.nowPose);
                xPower = txPID.calculate(now.getX(), trajPose.getX());
                yPower = -tyPID.calculate(now.getY(), trajPose.getY());
                drive.loopFC(now.getHeading(), xPower, yPower, 0); // ignore the warning, is because of wpilib coord system
            case SCORE_SPECIMEN:
                if (previousState != currentState) {
                    n.goSpecimenDown(2);
                    previousState = STATES.SCORE_SPECIMEN;
                } else if (n.isAtTargetHeight()) {
                    currentState = STATES.RELEASE_AND_PARK;
                }
                n.loop();
                break;
            case RELEASE_AND_PARK:
                if (previousState != currentState) {
                    n.release();
                    trajTimer.reset();
                    previousState = STATES.RELEASE_AND_PARK;
                }else if (checkTrajEndReached(trajSequence1)) {
                    currentState = STATES.STOP;
                }
                trajPose = getPoseAtTime(trajSequence1, trajTimer.seconds());
                now = rotateFTCLibPose(drive.nowPose);
                xPower = txPID.calculate(now.getX(), trajPose.getX());
                yPower = -tyPID.calculate(now.getY(), trajPose.getY());
                drive.loopFC(now.getHeading(), xPower, yPower, rotPower);
            case STOP:
                break;
                //hi oat you are hot
        }

        doTelemetry(telemetry);
        doTelemetry(t2);
        n.getTelemetry(t2);

    }
    public boolean checkTrajEndReached(TrajectorySequence sequence) {
        if (Math.abs(now.getX() - sequence.end().getX()) < 4 &&
                Math.abs(now.getY() - sequence.end().getY()) < 4 &&
                Math.abs(now.getHeading() - sequence.end().getHeading()) < 15) {
            return true;
        } else {
            return false;
        }
    }
    public void doTelemetry(Telemetry t) {
//        drive.getTelemetry(t);
        t.addData("STAGE", currentState);
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
