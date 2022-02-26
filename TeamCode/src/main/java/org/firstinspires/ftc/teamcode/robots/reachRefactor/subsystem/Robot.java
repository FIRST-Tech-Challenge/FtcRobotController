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

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;

/**
 * @author Mahesh Natamai
 */

@Config(value = "FFRobot")
public class Robot implements Subsystem {

    public static double BUCKET_TRIGGER_DISTANCE = 7;

    public DriveTrain driveTrain;
    public Turret turret;
    public Crane crane;
    public Gripper gripper;
    public Subsystem[] subsystems;

    private long[] subsystemUpdateTimes;

    private final List<LynxModule> hubs;

    private Articulation articulation;
    private final Map<Articulation, StateMachine> articulationMap;

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
        articulationMap.put(Articulation.TRANSFER, transfer);
        articulationMap.put(Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER, dumpAndSetCraneForTransfer);
        articulationMap.put(Articulation.GRAB_AND_TRANSFER, grabAndTransfer);

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

        if (gripper.getPitchTargetPos() == Gripper.PITCH_DOWN && gripper.getFreightDistance() < Gripper.FREIGHT_TRIGGER && articulation != Articulation.GRAB_AND_TRANSFER)
            articulation = Articulation.GRAB_AND_TRANSFER;

        if(crane.getShoulderTargetAngle() > 15 && crane.getElbowTargetAngle() > 75 && crane.getWristTargetAngle() > 15 && crane.getBucketDistance() < BUCKET_TRIGGER_DISTANCE && articulation != Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER)
            articulation = Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER;

        articulate(articulation);

        for(int i = 0; i < subsystems.length; i++) {
            Subsystem subsystem = subsystems[i];
            long updateStartTime = System.nanoTime();
            subsystem.update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }

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
        DUMP_AND_SET_CRANE_FOR_TRANSFER,
        GRAB_AND_TRANSFER,

        AUTO_HIGH_TIER_RED,
        AUTO_HIGH_TIER_BLUE,
        AUTO_MIDDLE_TIER_RED,
        AUTO_MIDDLE_TIER_BLUE,
        AUTO_LOW_TIER_RED,
        AUTO_LOW_TIER_BLUE
    }

    // Tele-Op articulations
    private StateMachine grabAndTransfer = getStateMachine(new Stage())
            .addSingleState(() -> driveTrain.setChassisLength(MIN_CHASSIS_LENGTH))
            .addSingleState(() -> gripper.articulate(Gripper.Articulation.LIFT))
            .addState(() -> gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .addSingleState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .addSingleState(() -> gripper.setIntakePower(1.0))
            .addSingleState(() -> gripper.articulate(Gripper.Articulation.TRANSFER))
            .addState(() -> gripper.getArticulation() == Gripper.Articulation.MANUAL)
            .addSingleState(() -> gripper.setIntakePower(0.0))
            .addSingleState(() -> crane.articulate(Crane.Articulation.HOME))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .build();

    private StateMachine dumpAndSetCraneForTransfer = getStateMachine(new Stage())
            .addTimedState(1f, () -> crane.dump(), () -> {})
            .addSingleState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
            .build();

    private StateMachine transfer = getStateMachine(new Stage())
            .addSingleState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addState(() -> crane.getArticulation() == Crane.Articulation.MANUAL)
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


    private StateMachine startEnd = getStateMachine(new Stage())
            .addTimedState(1f, () -> driveTrain.setDuckSpinnerPower(0.5), () -> driveTrain.setDuckSpinnerPower(0))
            .build();

    private boolean handleAutoCrane(Position targetPosition, double targetHeight, double hubRadius) {
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
        turret.setTargetHeading(wrapAngle(360 - Math.toDegrees(wrapAngleRad(turretAngle) - (pose.getHeading() + Math.toRadians(180)))));

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
            crane.setDumpPos(wristAngle + 180);
        }

        return crane.isDumping();
    }

    private StateMachine autoHighTierRed = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.RED_SHIPPING_HUB, HIGH_TIER_SHIPPING_HUB_HEIGHT, HIGH_TIER_RADIUS))
            .build();
    private StateMachine autoHighTierBlue = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.BLUE_SHIPPING_HUB, HIGH_TIER_SHIPPING_HUB_HEIGHT, HIGH_TIER_RADIUS))
            .build();
    private StateMachine autoMiddleTierRed = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.RED_SHIPPING_HUB, MIDDLE_TIER_SHIPPING_HUB_HEIGHT, MIDDLE_TIER_RADIUS))
            .build();
    private StateMachine autoMiddleTierBlue = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.BLUE_SHIPPING_HUB, MIDDLE_TIER_SHIPPING_HUB_HEIGHT, MIDDLE_TIER_RADIUS))
            .build();
    private StateMachine autoLowTierRed = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.RED_SHIPPING_HUB, LOW_TIER_SHIPPING_HUB_HEIGHT, LOW_TIER_RADIUS))
            .build();
    private StateMachine autoLowTierBlue = getStateMachine(new Stage())
            .addState(() -> handleAutoCrane(Position.BLUE_SHIPPING_HUB, LOW_TIER_SHIPPING_HUB_HEIGHT, LOW_TIER_RADIUS))
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
