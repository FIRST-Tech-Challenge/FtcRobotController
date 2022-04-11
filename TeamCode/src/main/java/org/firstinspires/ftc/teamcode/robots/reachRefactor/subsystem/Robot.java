package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;

import android.graphics.Bitmap;

/**
 * @author Mahesh Natamai
 */

@Config(value = "FFRobot")
public class Robot implements Subsystem {

    public DriveTrain driveTrain;
    public Turret turret;
    public Crane crane;
    public Gripper gripper;
    public Subsystem[] subsystems;

    private long[] subsystemUpdateTimes;
    private boolean autoDumpEnabled, doubleDuckEnabled;

    private final List<LynxModule> hubs;

    private Articulation articulation;
    private final Map<Articulation, StateMachine> articulationMap;

    private Bitmap craneBitmap;
    private Mat craneMat;
    public static int CB_WIDTH = 320;
    public static int CB_HEIGHT = 240;

    public Robot(HardwareMap hardwareMap, boolean simulated) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap, simulated);
        turret = new Turret(hardwareMap, simulated);
        crane = new Crane(hardwareMap, turret, simulated);
        gripper = new Gripper(hardwareMap, simulated);

        subsystems = new Subsystem[] {driveTrain, crane, gripper};
        subsystemUpdateTimes = new long[subsystems.length];

        articulation = Articulation.MANUAL;

        articulationMap = new HashMap<>();
        articulationMap.put(Articulation.INIT, init);
        articulationMap.put(Articulation.START, start);
        articulationMap.put(Articulation.START_END_GAME, startEnd);
        articulationMap.put(Articulation.START_DOWN, startDown);

        articulationMap.put(Articulation.TRANSFER, transfer);
        articulationMap.put(Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER, dumpAndSetCraneForTransfer);
        articulationMap.put(Articulation.GRAB_AND_TRANSFER, grabAndTransfer);

        articulationMap.put(Articulation.AUTO_HIGH_TIER_RED, autoHighTierRed);
        articulationMap.put(Articulation.AUTO_HIGH_TIER_BLUE, autoHighTierBlue);

        articulationMap.put(Articulation.DOUBLE_DUCK_DUMP_AND_SET_CRANE_FOR_TRANSFER, doubleDuckDumpAndSetCraneForTransfer);
        articulationMap.put(Articulation.DOUBLE_DUCK_GRAB_AND_TRANSFER, doubleDuckGrabAndTransfer);

        craneBitmap = Bitmap.createBitmap(CB_WIDTH, CB_HEIGHT, Bitmap.Config.RGB_565);
        craneMat = new Mat(CB_HEIGHT, CB_WIDTH, CvType.CV_8UC3);
        autoDumpEnabled = true;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);
        telemetryMap.put("auto-dump enabled", autoDumpEnabled);
        if(debug) {
            for (int i = 0; i < subsystems.length; i++) {
                String name = subsystems[i].getClass().getSimpleName();
                telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
            }
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Robot";
    }

    @Override
    public void update(Canvas fieldOverlay) {
        for (LynxModule module : hubs)
            module.clearBulkCache();

        if (gripper.getPitchTargetPos() == Gripper.PITCH_DOWN &&
                gripper.getTargetPos() == Gripper.OPEN &&
                gripper.getFreightDistance() < Gripper.FREIGHT_TRIGGER &&
                (doubleDuckEnabled ? articulation != Articulation.DOUBLE_DUCK_GRAB_AND_TRANSFER : articulation != Articulation.GRAB_AND_TRANSFER))
            articulation = doubleDuckEnabled ? Articulation.DOUBLE_DUCK_GRAB_AND_TRANSFER : Articulation.GRAB_AND_TRANSFER;

        articulate(articulation);

        for(int i = 0; i < subsystems.length; i++) {
            Subsystem subsystem = subsystems[i];
            long updateStartTime = System.nanoTime();
            subsystem.update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }

        double theta1 = wrapAngleRad(Math.toRadians(90 - crane.getShoulderTargetAngle()));
        double theta2 = -wrapAngleRad(Math.toRadians(180 - crane.getElbowTargetAngle()));
        double height = SHOULDER_TO_ELBOW * Math.sin(theta1) + ELBOW_TO_WRIST * Math.sin(theta1 + theta2);

        if(autoDumpEnabled && height > HIGH_TIER_SHIPPING_HUB_HEIGHT - SHOULDER_AXLE_TO_GROUND_HEIGHT && crane.getBucketDistance() < Crane.BUCKET_TRIGGER_DISTANCE && articulation == Articulation.MANUAL && crane.getArticulation() == Crane.Articulation.MANUAL)
            articulation = Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER;

        double wristAngle = wrapAngleRad(Math.toRadians(180) - wrapAngleRad(-(theta1 + theta2) + Math.toRadians(crane.getWristTargetAngle())));

        double x = (CB_WIDTH / 2.0);
        double y = (CB_HEIGHT / 2.0);
        double x1 = x + 70.1575 * Math.cos(theta1);
        double y1 = y + 70.1575 * Math.sin(theta1);
        double x2 = x1 + 55.118 * Math.cos(theta1 + theta2);
        double y2 = y1 + 55.118 * Math.sin(theta1 + theta2);
        double x3 = x2 + 10 * Math.cos(wristAngle);
        double y3 = y2 + 10 * Math.sin(wristAngle);
        double x4 = x2 - 10 * Math.cos(wristAngle);
        double y4 = y2 - 10 * Math.sin(wristAngle);

        craneMat.setTo(new Scalar(0));
        Imgproc.line(craneMat, new Point(x, CB_HEIGHT - y), new Point(x1, CB_HEIGHT - y1), new Scalar(255, 255, 255), 2);
        Imgproc.line(craneMat, new Point(x1, CB_HEIGHT - y1), new Point(x2, CB_HEIGHT - y2), new Scalar(255, 255, 255), 2);
        Imgproc.line(craneMat, new Point(x2, CB_HEIGHT - y2), new Point(x3, CB_HEIGHT - y3), new Scalar(255, 255, 255), 2);
        Imgproc.line(craneMat, new Point(x2, CB_HEIGHT - y2), new Point(x4, CB_HEIGHT - y4), new Scalar(255, 255, 255), 2);

        Utils.matToBitmap(craneMat, craneBitmap);

        DashboardUtil.drawRobot(fieldOverlay, driveTrain.getPoseEstimate(), driveTrain.getChassisLength(), driveTrain.getSwivelAngle(), driveTrain.getWheelVelocities(), turret.getTargetHeading(), crane.getShoulderTargetAngle(), crane.getElbowTargetAngle(), crane.getWristTargetAngle());
    }

    public Bitmap getBitmap() {
        return craneBitmap;
    }

    @Override
    public void stop() {
        for(Subsystem subsystem: subsystems)
            subsystem.stop();
    }

    //----------------------------------------------------------------------------------------------
    // Articulations
    //----------------------------------------------------------------------------------------------

    public enum Articulation {
        MANUAL,

        // misc. articulations
        INIT,
        START,
        START_DOWN, // use to prep for start - stows the crane
        START_END_GAME, //use on a timer to automatically deploy carousel spinner 10 seconds before end game

        // tele-op articulations
        TRANSFER,
        DUMP_AND_SET_CRANE_FOR_TRANSFER,
        GRAB_AND_TRANSFER,

        AUTO_HIGH_TIER_RED,
        AUTO_HIGH_TIER_BLUE,
        AUTO_MIDDLE_TIER_RED,
        AUTO_MIDDLE_TIER_BLUE,
        AUTO_LOW_TIER_RED,
        AUTO_LOW_TIER_BLUE,

        DOUBLE_DUCK_GRAB_AND_TRANSFER,
        DOUBLE_DUCK_DUMP_AND_SET_CRANE_FOR_TRANSFER
    }

    // Tele-Op articulations
    private StateMachine doubleDuckGrabAndTransfer = getStateMachine(new Stage())
            .addSingleState(() -> driveTrain.setChassisLength(MIN_CHASSIS_LENGTH))
            .addSingleState(() -> gripper.articulate(Gripper.Articulation.LIFT))
            .addSingleState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL && gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .addSingleState(() -> gripper.articulate(Gripper.Articulation.TRANSFER))
            .addState(() -> gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .addSingleState(() -> crane.articulate(Crane.Articulation.HOME))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .addSingleState(() -> driveTrain.setChassisLength(MAX_CHASSIS_LENGTH))
            .build();

    private StateMachine doubleDuckDumpAndSetCraneForTransfer = getStateMachine(new Stage())
            .addSingleState(() -> crane.setToHomeEnabled(false))
            .addTimedState(.5f, () -> crane.dump(), () -> {})
//            .addSingleState(() -> { crane.setToHomeEnabled(false); })
//            .addSingleState(() -> crane.articulate(Crane.Articulation.POST_DUMP))
//            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .addSingleState(() -> crane.articulate(Crane.Articulation.POST_DUMP))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .addSingleState(() -> crane.articulate(Crane.Articulation.HOME))
            .addTimedState(1f, () -> {}, () -> {})
            .addSingleState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
//            .addSingleState(() -> { crane.setToHomeEnabled(true); })
            .addSingleState(() -> crane.setToHomeEnabled(true))
            .addSingleState(() -> gripper.setDuck())
            .addSingleState(() -> driveTrain.setChassisLength(MIN_CHASSIS_LENGTH))
            .addState(() -> gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .build();



    private Stage grabAndTransferStage = new Stage();
    private StateMachine grabAndTransfer = getStateMachine(grabAndTransferStage)
            .addSingleState(() -> driveTrain.setChassisLength(MIN_CHASSIS_LENGTH))
            .addSingleState(() -> gripper.articulate(Gripper.Articulation.LIFT))
            .addSingleState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL && gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .addSingleState(() -> gripper.articulate(Gripper.Articulation.TRANSFER))
            .addState(() -> gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .addSingleState(() -> crane.articulate(Crane.Articulation.HOME))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .build();

    private StateMachine dumpAndSetCraneForTransfer = getStateMachine(new Stage())
            .addSingleState(() -> crane.setToHomeEnabled(false))
            .addTimedState(.5f, () -> crane.dump(), () -> {})
//            .addSingleState(() -> { crane.setToHomeEnabled(false); })
//            .addSingleState(() -> crane.articulate(Crane.Articulation.POST_DUMP))
//            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .addSingleState(() -> crane.articulate(Crane.Articulation.POST_DUMP))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .addSingleState(() -> crane.articulate(Crane.Articulation.HOME))
            .addTimedState(1f, () -> {}, () -> {})
            .addSingleState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
//            .addSingleState(() -> { crane.setToHomeEnabled(true); })
            .addSingleState(() -> crane.setToHomeEnabled(true))
            .build();

    private StateMachine transfer = getStateMachine(new Stage())
            .addSingleState(() -> driveTrain.setChassisLength(MIN_CHASSIS_LENGTH))
            .addSingleState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .addSingleState(() -> gripper.articulate(Gripper.Articulation.TRANSFER))
            .addState(() -> gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .addSingleState(() -> gripper.setIntakePower(1.0))
            .addSingleState(() -> gripper.articulate(Gripper.Articulation.TRANSFER))
            .addState(() -> gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .addSingleState(() -> gripper.setIntakePower(0.0))
            .addSingleState(() -> crane.articulate(Crane.Articulation.HOME))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .build();

    // Game articulations
    private StateMachine init = getStateMachine(new Stage())
            .addSingleState(() -> gripper.lift())
            .addSingleState(() -> crane.articulate(Crane.Articulation.HOME))
            .addSimultaneousStates(
                    () -> gripper.getArticulation() == Gripper.Articulation.MANUAL,
                    () -> crane.getArticulation() == Crane.Articulation.MANUAL
            )
            .build();

    private StateMachine start = getStateMachine(new Stage())
            .addSingleState(() -> gripper.lift())
            .addSingleState(() -> crane.articulate(Crane.Articulation.HOME))
            .addSimultaneousStates(
                    () -> gripper.getArticulation() == Gripper.Articulation.MANUAL,
                    () -> crane.getArticulation() == Crane.Articulation.MANUAL
            )
            .build();

    private StateMachine startDown = getStateMachine(new Stage())
            .addSingleState(() -> crane.articulate(Crane.Articulation.INIT))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .build();

    private StateMachine startEnd = getStateMachine(new Stage())
            .addTimedState(1f, () -> driveTrain.setDuckSpinnerPower(0.5), () -> driveTrain.setDuckSpinnerPower(0))
            .build();

    public boolean handleAutoCrane(Pose2d targetPosition, double targetHeight) {
        Pose2d pose = driveTrain.getPoseEstimate();
        Vector2d turretPose = pose.vec().minus(
                new Vector2d(
                        driveTrain.getChassisLength(),
                        0
                ).rotated(pose.getHeading())
        );

        Vector2d shippingHub = targetPosition.vec();
        Vector2d diff = shippingHub.minus(turretPose);
        double turretAngle = Math.atan2(diff.getY(), diff.getX());
        turret.setTargetHeading(wrapAngle(360 - Math.toDegrees(wrapAngleRad(turretAngle) - (pose.getHeading() + Math.toRadians(180)))));

        double dx = Math.hypot(diff.getX(), diff.getY());
        double dy = targetHeight - SHOULDER_AXLE_TO_GROUND_HEIGHT;

        double[] angles = craneIK(dx, dy);

        if(angles != null) {
            crane.setShoulderTargetAngle(angles[0]);
            crane.setElbowTargetAngle(angles[1]);
            crane.setWristTargetAngle(angles[2]);
            crane.setDumpPos(wrapAngle(angles[3]));
        }

        return crane.isDumping();
    }

    private StateMachine autoHighTierRed = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.RED_SHIPPING_HUB.getPose(), HIGH_TIER_SHIPPING_HUB_HEIGHT + 5))
            .build();
    private StateMachine autoHighTierBlue = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.BLUE_SHIPPING_HUB.getPose(), HIGH_TIER_SHIPPING_HUB_HEIGHT + 5))
            .build();

    public boolean articulate(Articulation articulation) {
        if(articulation.equals(Articulation.MANUAL))
            return true;
        this.articulation = articulation;
        if(articulationMap.get(articulation).execute()) {
            this.articulation = Articulation.MANUAL;
            return true;
        }
        return false;
    }

    public void setAutoDumpEnabled(boolean autoDumpEnabled) {
        this.autoDumpEnabled = autoDumpEnabled;
    }

    public boolean isAutoDumpEnabled() {
        return autoDumpEnabled;
    }

    public void setDoubleDuckEnabled(boolean doubleDuckEnabled) {
        this.doubleDuckEnabled = doubleDuckEnabled;
    }

    public boolean isDoubleDuckEnabled() {
        return doubleDuckEnabled;
    }
}
