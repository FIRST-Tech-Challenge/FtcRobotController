package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;

/**
 * @author Mahesh Natamai
 */

@Config
public class Robot implements Subsystem {
    public DriveTrain driveTrain;
    public Turret turret;
    public Crane crane;
    public Gripper gripper;
    public Subsystem[] subsystems;

    private final List<LynxModule> hubs;

    private Articulation articulation;
    private final Map<Articulation, StateMachine> articulationMap;

    private double theta1, theta2;

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

        articulation = Articulation.MANUAL;

        articulationMap = new HashMap<>();
        articulationMap.put(Articulation.INIT, init);
        articulationMap.put(Articulation.START, start);
        articulationMap.put(Articulation.START_END_GAME, startEnd);
        articulationMap.put(Articulation.TRANSFER, transfer);
        articulationMap.put(Articulation.TRANSFER_AND_HIGH_TIER, transferAndHighTier);
        articulationMap.put(Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER, dumpAndSetCraneForTransfer);
        articulationMap.put(Articulation.AUTO_GRAB_AND_TRANSFER, autoGrabAndTransfer);

        articulationMap.put(Articulation.AUTO_HIGH_TIER_RED, autoHighTierRed);
        articulationMap.put(Articulation.AUTO_HIGH_TIER_BLUE, autoHighTierBlue);
        articulationMap.put(Articulation.AUTO_MIDDLE_TIER_RED, autoMiddleTierRed);
        articulationMap.put(Articulation.AUTO_MIDDLE_TIER_BLUE, autoMiddleTierBlue);
        articulationMap.put(Articulation.AUTO_LOW_TIER_RED, autoLowTierRed);
        articulationMap.put(Articulation.AUTO_LOW_TIER_BLUE, autoLowTierBlue);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);
        telemetryMap.put("theta1", theta1);
        telemetryMap.put("theta2", theta2);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Robot";
    }

    @Override
    public void update(Canvas fieldOverlay) {
        for (LynxModule module : hubs) {
            module.clearBulkCache();
        }
        for(Subsystem subsystem: subsystems)
            subsystem.update(fieldOverlay);

        articulate(articulation);

        if (gripper.getPitchTargetPos() < 1500 && gripper.getFreightDistance() < Gripper.FREIGHT_TRIGGER && gripper.articulation == Gripper.Articulation.MANUAL)
            articulate(Articulation.AUTO_GRAB_AND_TRANSFER);

        DashboardUtil.drawRobot(fieldOverlay, driveTrain.getPoseEstimate(), driveTrain.getChassisLength(), driveTrain.getSwivelAngle(), driveTrain.getWheelVelocities(), turret.getTargetHeading(), crane.getShoulderTargetAngle(), crane.getElbowTargetAngle(), crane.getWristTargetAngle());
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
        START, //use to prep for start - mostly stows the Crane
        START_END_GAME, //use on a timer to automatically deploy carousel spinner 10 seconds before end game

        // tele-op articulations
        TRANSFER,
        TRANSFER_AND_HIGH_TIER,
        DUMP_AND_SET_CRANE_FOR_TRANSFER,
        AUTO_GRAB_AND_TRANSFER,
        AUTO_HIGH_TIER_RED,
        AUTO_HIGH_TIER_BLUE,
        AUTO_MIDDLE_TIER_RED,
        AUTO_MIDDLE_TIER_BLUE,
        AUTO_LOW_TIER_RED,
        AUTO_LOW_TIER_BLUE
    }

    // Tele-Op articulations
    private StateMachine transferAndHighTier = getStateMachine(new Stage())
            .addState(() -> {
                driveTrain.setChassisLength(MIN_CHASSIS_LENGTH);
                return driveTrain.chassisLengthOnTarget();
            })
            .addState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addTimedState(1f, () -> gripper.articulate(Gripper.Articulation.TRANSFER), () -> {})
            .addState(() -> crane.articulate(Crane.Articulation.HIGH_TIER))
            .build();

    private StateMachine autoGrabAndTransfer = getStateMachine(new Stage())
            .addState(() -> gripper.articulate(Gripper.Articulation.LIFT))
            .addTimedState(1f, () -> {}, () -> {})
            .addConditionalState(() -> crane.getArticulation() == Crane.Articulation.TRANSFER, (() -> gripper.articulate(Gripper.Articulation.TRANSFER)), (() -> true))
            .addTimedState(1f, () -> {}, () -> {})
            .addConditionalState(() -> crane.getArticulation() == Crane.Articulation.TRANSFER, (() -> crane.articulate(Crane.Articulation.HOME)), (() -> true) )
            .build();

    private StateMachine dumpAndSetCraneForTransfer = getStateMachine(new Stage())
//            .addConditionalState(() -> crane.getArticulation() != Crane.Articulation.HOME, () -> crane.dump(), () -> true)
            .addSingleState(() -> crane.dump())
            .addTimedState(1f, () -> {}, () -> {})
            .addState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .build();

    private StateMachine transfer = getStateMachine(new Stage())
            .addState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addSingleState(() -> gripper.setIntakePower(1.0))
            .addTimedState(1f, () -> gripper.articulate(Gripper.Articulation.TRANSFER), () -> {})
            .addSingleState(() -> gripper.setIntakePower(0.0))
            .addState(() -> crane.articulate(Crane.Articulation.HOME))
            .build();

    private StateMachine init = getStateMachine(new Stage())
            .addSingleState(() -> gripper.lift())
            .addState(() -> crane.articulate(Crane.Articulation.HOME)) //for visual confirmation that the crane is vertial and aligned
            .build();

    private StateMachine start = getStateMachine(new Stage())
            .addSingleState(() -> gripper.lift())
            .addState(() -> crane.articulate(Crane.Articulation.HOME))
            .build();

    private StateMachine startEnd = getStateMachine(new Stage())
            .addTimedState(2, () -> driveTrain.setDuckSpinnerPower(0.5), () -> driveTrain.setDuckSpinnerPower(0))
            .build();

    private boolean handleAutoCrane(Position targetPosition, double targetHeight) {
        Pose2d pose = driveTrain.getPoseEstimate();
        Vector2d turretPose = pose.vec().minus(
                new Vector2d(
                        driveTrain.getChassisLength(),
                        0
                ).rotated(pose.getHeading())
        );

        Vector2d shippingHub = targetPosition.getPose().vec();
        Vector2d diff = shippingHub.minus(turretPose);
        double turretAngle = Math.atan2(diff.getY(), diff.getX());
        turret.setTargetHeading(wrapAngle(180 - Math.toDegrees(wrapAngleRad(turretAngle) - (pose.getHeading() + Math.toRadians(180)))));

        double dx = Math.hypot(diff.getX(), diff.getY());
        double dy = targetHeight - SHOULDER_AXLE_TO_GROUND_HEIGHT;

        double theta2 = -Math.acos((Math.pow(dx, 2) + Math.pow(dy, 2) - Math.pow(SHOULDER_TO_ELBOW, 2) - Math.pow(ELBOW_TO_WRIST, 2)) / (2 * SHOULDER_TO_ELBOW * ELBOW_TO_WRIST));
        double theta1 = Math.atan2(dy, dx) - Math.atan2(ELBOW_TO_WRIST * Math.sin(theta2), SHOULDER_TO_ELBOW + ELBOW_TO_WRIST * Math.cos(theta2));

        if(!Double.isNaN(theta1) && !Double.isNaN(theta2)) {
            double shoulderTargetAngle = wrapAngle(90 - Math.toDegrees(theta1));
            double elbowTargetAngle = 180 - wrapAngle(Math.toDegrees(-theta2));
            double wristAngle = 90 - (wrapAngle(Math.toDegrees(-theta2)) - wrapAngle(Math.toDegrees(theta1)));

            crane.setShoulderTargetAngle(shoulderTargetAngle);
            crane.setElbowTargetAngle(elbowTargetAngle);
            crane.setWristTargetAngle(wristAngle);
            crane.setDumpPos(wrapAngle(wristAngle + 180));
        }

        return crane.isDumping();
    }

    private StateMachine autoHighTierRed = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.RED_SHIPPING_HUB, HIGH_TIER_SHIPPING_HUB_HEIGHT))
            .build();
    private StateMachine autoHighTierBlue = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.BLUE_SHIPPING_HUB, HIGH_TIER_SHIPPING_HUB_HEIGHT))
            .build();
    private StateMachine autoMiddleTierRed = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.RED_SHIPPING_HUB, MIDDLE_TIER_SHIPPING_HUB_HEIGHT))
            .build();
    private StateMachine autoMiddleTierBlue = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.BLUE_SHIPPING_HUB, MIDDLE_TIER_SHIPPING_HUB_HEIGHT))
            .build();
    private StateMachine autoLowTierRed = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.RED_SHIPPING_HUB, LOW_TIER_SHIPPING_HUB_HEIGHT))
            .build();
    private StateMachine autoLowTierBlue = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.BLUE_SHIPPING_HUB, LOW_TIER_SHIPPING_HUB_HEIGHT))
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
}
