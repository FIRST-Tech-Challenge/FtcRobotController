package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

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
    // Subsystems
    public DriveTrain driveTrain;
    public Crane crane;
    public Gripper gripper;
    public Subsystem[] subsystems;

    // State
    private Constants.Alliance alliance;
    private Articulation articulation;

    // Constants
    private static final String TELEMETRY_NAME = "Robot";
    public static String AXLE_STROKE_COLOR = "Black";
    public static String TURN_RADIUS_STROKE_COLOR = "Crimson";
    public static String WHEEL_STROKE_COLOR = "SpringGreen";
    public static double DOTTED_LINE_DASH_LENGTH = 1; // in inches

    public static double DEFAULT_TARGET_DISTANCE =  Constants.MIN_CHASSIS_LENGTH + (Constants.MAX_CHASSIS_LENGTH - Constants.MIN_CHASSIS_LENGTH) / 3;

    public Robot(HardwareMap hardwareMap) {
        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap);
        crane = new Crane(hardwareMap);
        gripper = new Gripper(hardwareMap);
        subsystems = new Subsystem[] {driveTrain, crane, gripper};

        articulation = Articulation.MANUAL;
    }

    public void drawFieldOverlay(TelemetryPacket packet) {
        Canvas fieldOverlay = packet.fieldOverlay();

        SimpleMatrix pose = driveTrain.getPose();

        SimpleMatrix position = pose.rows(0, 2);
        double heading = pose.get(2);

        // calculating wheel positions
        SimpleMatrix leftWheel = new SimpleMatrix(new double[][] {{ -Constants.TRACK_WIDTH / 2 , 0 }});
        SimpleMatrix rightWheel = new SimpleMatrix(new double[][] {{ Constants.TRACK_WIDTH / 2, 0 }});
        SimpleMatrix swerveWheel = new SimpleMatrix(new double[][] {{ 0, -driveTrain.getChassisDistance() }});

        leftWheel = position.plus(UtilMethods.rotateVector(leftWheel, heading)).scale(Constants.INCHES_PER_METER);
        rightWheel = position.plus(UtilMethods.rotateVector(rightWheel, heading)).scale(Constants.INCHES_PER_METER);
        swerveWheel = position.plus(UtilMethods.rotateVector(swerveWheel, heading)).scale(Constants.INCHES_PER_METER);

        // drawing axles
        CanvasUtils.drawLine(fieldOverlay, leftWheel, rightWheel, AXLE_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, leftWheel.plus(rightWheel).divide(2), swerveWheel, AXLE_STROKE_COLOR);

        // drawing wheel vectors
        SimpleMatrix genericWheelVector = new SimpleMatrix(new double[][]
                {{ 0, Constants.WHEEL_RADIUS * 2 * Constants.INCHES_PER_METER }}
        );
        CanvasUtils.drawLine(fieldOverlay, leftWheel, leftWheel.plus(
            UtilMethods.rotateVector(
                genericWheelVector,
                heading
            )
        ), WHEEL_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, rightWheel, rightWheel.plus(
            UtilMethods.rotateVector(
                genericWheelVector,
                heading
            )
        ), WHEEL_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, swerveWheel, swerveWheel.plus(
            UtilMethods.rotateVector(
                UtilMethods.rotateVector(
                    genericWheelVector,
                    heading
                ).transpose(),
                driveTrain.getSwivelAngle()
            )
        ), WHEEL_STROKE_COLOR);



        // calculating the instantaneous center of rotation
        double turnRadius = driveTrain.getTurnRadius();
        SimpleMatrix ICC = new SimpleMatrix(new double[][] {{ -turnRadius, 0 }});
        ICC = UtilMethods.rotateVector(ICC, heading).scale(Constants.INCHES_PER_METER);

        // drawing ICC and turn radius
        CanvasUtils.drawDottedLine(fieldOverlay, ICC, position.scale(Constants.INCHES_PER_METER), TURN_RADIUS_STROKE_COLOR, DOTTED_LINE_DASH_LENGTH);
        fieldOverlay.strokeCircle(ICC.get(0), ICC.get(1), Math.abs(turnRadius * Constants.INCHES_PER_METER));
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
        START,
        MANUAL,
        // tele-op articulations

        TRANSFER,
    }

    // Misc. Articulations
    private Stage startStage = new Stage();
    private StateMachine start = UtilMethods.getStateMachine(startStage)
            .addTimedState(1f, () -> {
                gripper.pitchGripper(true);
                gripper.actuateGripper(true);
            }, () -> {})
            .addTimedState(1f, () -> gripper.actuateGripper(false), () -> {})
            .addTimedState(2f, () -> driveTrain.handleDuckSpinner(-0.5), () -> driveTrain.handleDuckSpinner(0))
            .addState(() -> crane.articulate(Crane.Articulation.HOME))
            .addSingleState(() -> driveTrain.setTargetChassisDistance(DEFAULT_TARGET_DISTANCE))
            .addSingleState(() -> driveTrain.setMaintainChassisDistanceEnabled(true))
            .build();

    // Tele-Op articulations
    private Stage transferStage = new Stage();
    private StateMachine transfer = UtilMethods.getStateMachine(transferStage)
            .addState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addTimedState(1, () -> {
                gripper.pitchGripper(true);
                gripper.toggleGripper();
            }, () -> {
                gripper.actuateGripper(false);
                gripper.pitchGripper(false);
            })
            .addTimedState(1, () -> {}, () -> {})
            .build();

    public boolean articulate(Articulation articulation) {
        this.articulation = articulation;

        switch(articulation) {
            case MANUAL:
                return true;
            case START:
                if(start.execute()) {
                    this.articulation = Articulation.MANUAL;
                    return true;
                }
                break;
            case TRANSFER:
                if(transfer.execute()){
                    this.articulation = Articulation.MANUAL;
                    return true;
                }
                break;
        }
        return false;
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

    public Constants.Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Constants.Alliance alliance) {
        this.alliance = alliance;
    }


}
