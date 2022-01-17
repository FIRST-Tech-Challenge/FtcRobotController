package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.CanvasUtils;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.HashMap;
import java.util.Map;

@Config
public class Robot implements Subsystem {
    // subsystems
    public DriveTrain driveTrain;
    public Crane crane;
    public Gripper gripper;
    public Subsystem[] subsystems;

    // sensors
    public DigitalChannel freightSensor;


    // state
    private Articulation articulation;

    // constants
    private static final String TELEMETRY_NAME = "Robot";
    public static String AXLE_STROKE_COLOR = "Black";
    public static String WHEEL_STROKE_COLOR = "SpringGreen";
    public static double INCHES_PER_METER = 39.3701;

    private Map<Articulation, StateMachine> articulationMap;

    public Robot(HardwareMap hardwareMap) {
        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap);
        crane = new Crane(hardwareMap);
        gripper = new Gripper(hardwareMap);
        subsystems = new Subsystem[] {driveTrain, crane, gripper};

        articulation = Articulation.MANUAL;

        articulationMap = new HashMap<>();
        articulationMap.put(Articulation.SELFTEST, selftest);
        articulationMap.put(Articulation.START, start);
        articulationMap.put(Articulation.LEGALSTARTPOS, startpos);
        articulationMap.put(Articulation.DIAGNOSTIC, diagnostic);
        articulationMap.put(Articulation.TRANSFER, transfer);
    }

    public void drawFieldOverlay(TelemetryPacket packet) {
        Canvas fieldOverlay = packet.fieldOverlay();

        double[] pose = driveTrain.getPose();

        SimpleMatrix position = new SimpleMatrix(new double[][] {{ pose[0], pose[1] }});
        double heading = pose[2];

        // calculating wheel positions
        SimpleMatrix leftWheel = new SimpleMatrix(new double[][] {{ -Constants.TRACK_WIDTH / 2 , 0 }});
        SimpleMatrix rightWheel = new SimpleMatrix(new double[][] {{ Constants.TRACK_WIDTH / 2, 0 }});
        SimpleMatrix swerveWheel = new SimpleMatrix(new double[][] {{ 0, -driveTrain.getChassisDistance() }});

        leftWheel = position.plus(UtilMethods.rotateVector(leftWheel, Math.toRadians(heading))).scale(INCHES_PER_METER);
        rightWheel = position.plus(UtilMethods.rotateVector(rightWheel, Math.toRadians(heading))).scale(INCHES_PER_METER);
        swerveWheel = position.plus(UtilMethods.rotateVector(swerveWheel, Math.toRadians(heading))).scale(INCHES_PER_METER);

        // drawing axles
        CanvasUtils.drawLine(fieldOverlay, leftWheel, rightWheel, AXLE_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, leftWheel.plus(rightWheel).divide(2), swerveWheel, AXLE_STROKE_COLOR);

        // drawing wheel vectors
        SimpleMatrix genericWheelVector = new SimpleMatrix(new double[][]
                {{ 0, Constants.WHEEL_RADIUS * 2 * INCHES_PER_METER }}
        );
        CanvasUtils.drawLine(fieldOverlay, leftWheel, leftWheel.plus(
            UtilMethods.rotateVector(
                genericWheelVector, Math.toRadians(heading)
            )
        ), WHEEL_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, rightWheel, rightWheel.plus(
            UtilMethods.rotateVector(
                genericWheelVector, Math.toRadians(heading)
            )
        ), WHEEL_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, swerveWheel, swerveWheel.plus(
            UtilMethods.rotateVector(
                UtilMethods.rotateVector(
                    genericWheelVector,
                        Math.toRadians(heading)
                ), Math.toRadians(driveTrain.getSwivelAngle())
            )
        ), WHEEL_STROKE_COLOR);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("Articulation", articulation);

        return telemetryMap;
    }


    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void update() {
        for(Subsystem subsystem: subsystems)
            subsystem.update();

        articulate(articulation);
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
        SELFTEST,
        LEGALSTARTPOS,
        DIAGNOSTIC,
        START,

        // tele-op articulations
        TRANSFER,
    }

    // Misc. Articulations
    private Stage selfTest = new Stage();
    private StateMachine selftest = UtilMethods.getStateMachine(selfTest)
            .addSingleState(() -> gripper.Set())
            .addState(() -> crane.articulate(Crane.Articulation.INIT))
            .build();

    private Stage legalStartPos = new Stage();
    private StateMachine startpos = UtilMethods.getStateMachine(legalStartPos)
            //todo - move deploy duckspinner to end of auton:
            // .addTimedState(.75f, () -> driveTrain.handleDuckSpinner(-0.5), () -> driveTrain.handleDuckSpinner(0))
            .addState(() -> crane.articulate(Crane.Articulation.SIZING)) //fold the crane
            .addSingleState(()-> gripper.Lift()) //lift the gripper to vertical
//            .addSingleState(() -> driveTrain.setTargetChassisDistance(DEFAULT_TARGET_DISTANCE))
            .addSingleState(() -> driveTrain.setMaintainChassisDistanceEnabled(false)) //todo-make this false when ready to fix maintainChassisDistance
            .build();

    private Stage diagnosticStage = new Stage();
    private StateMachine diagnostic = UtilMethods.getStateMachine(diagnosticStage)
            // testing drivetrain
            .addTimedState(3f, () -> {
                driveTrain.drive(0.25, 0);
            }, () -> {})
            .addTimedState(3f, () -> {
                driveTrain.drive(0, Math.PI / 2);
            }, () -> {})

            // testing crane
            .addState(() -> crane.articulate(Crane.Articulation.HOME))
            .addState(() -> crane.articulate(Crane.Articulation.VALIDATE_ELBOW90))
            .addState(() -> crane.articulate(Crane.Articulation.VALIDATE_SHOULDER90))
            .addState(() -> crane.articulate(Crane.Articulation.VALIDATE_TURRET90R))
            .addState(() -> crane.articulate(Crane.Articulation.VALIDATE_TURRET90L))
            //.addState(() -> crane.articulate(Crane.Articulation.LOWEST_TIER))
            .addState(() -> crane.articulate(Crane.Articulation.HIGH_TIER))
            //.addState(() -> crane.articulate(Crane.Articulation.MIDDLE_TIER))
            //.addState(() -> crane.articulate(Crane.Articulation.STARTING))
            //.addState(() -> crane.articulate(Crane.Articulation.CAP))
            //.addState(() -> articulate(Robot.Articulation.TRANSFER))

            // testing gripper
            .addState(() -> crane.articulate(Crane.Articulation.HOME))
            .addTimedState(3f, () -> {
                gripper.togglePitch();
            }, () -> {})
            .addTimedState(3f, () -> {
                gripper.toggleGripper();
            }, () -> {})

            // testing turret
            .addTimedState(3f,
                    () -> crane.turret.setTargetAngle(90),
                    () -> {})
            .addTimedState(3f,
                    () -> crane.turret.setTargetAngle(180),
                    () -> {})
            .addTimedState(3f,
                    () -> crane.turret.setTargetAngle(270),
                    () -> {})
            .addTimedState(3f,
                    () -> crane.turret.setTargetAngle(360),
                    () -> {})
            .build();

    // Tele-Op articulations
    private Stage transferStage = new Stage();
    private StateMachine transfer = UtilMethods.getStateMachine(transferStage)
//            .addState(() -> {
//                        driveTrain.setTargetChassisDistance(Constants.MIN_CHASSIS_LENGTH);
//                        return driveTrain.chassisDistanceOnTarget();
//            })
            .addTimedState(1f, () -> crane.articulate(Crane.Articulation.TRANSFER), () -> gripper.articulate(Gripper.Articulation.TRANSFER))
            .build();

    private Stage startStage = new Stage();
    private StateMachine start = UtilMethods.getStateMachine(startStage)
            .addTimedState(2, () -> driveTrain.handleDuckSpinner(0.5), () -> driveTrain.handleDuckSpinner(0))
            .build();

    public boolean articulate(Articulation articulation) {
        this.articulation = articulation;

        if(articulation.equals(Articulation.MANUAL))
            return true;
        else if(articulationMap.get(articulation).execute()) {
            this.articulation = Articulation.MANUAL;
            return true;
        }
        return false;
    }
}
