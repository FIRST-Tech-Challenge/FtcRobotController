package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.CanvasUtils;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.MathUtils;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.HashMap;
import java.util.Map;

public class Robot implements Subsystem {
    // Subsystems
    public DriveTrain driveTrain;
    public Crane crane;
    public Gripper gripper;
    public Subsystem[] subsystems;

    // State
    private Constants.Alliance alliance;
    private Articulation articulation;

    private static final String TELEMETRY_NAME = "Robot";

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

        leftWheel = position.plus(MathUtils.rotateVector(leftWheel, heading)).scale(Constants.INCHES_PER_METER);
        rightWheel = position.plus(MathUtils.rotateVector(rightWheel, heading)).scale(Constants.INCHES_PER_METER);
        swerveWheel = position.plus(MathUtils.rotateVector(swerveWheel, heading)).scale(Constants.INCHES_PER_METER);

        // drawing axles
        CanvasUtils.drawLine(fieldOverlay, leftWheel, rightWheel, Constants.AXLE_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, leftWheel.plus(rightWheel).divide(2), swerveWheel, Constants.AXLE_STROKE_COLOR);

        // drawing wheel vectors
        SimpleMatrix genericWheelVector = new SimpleMatrix(new double[][]
                {{ 0, Constants.WHEEL_RADIUS * 2 * Constants.INCHES_PER_METER }}
        );
        CanvasUtils.drawLine(fieldOverlay, leftWheel, leftWheel.plus(
            MathUtils.rotateVector(
                genericWheelVector,
                heading
            )
        ), Constants.WHEEL_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, rightWheel, rightWheel.plus(
            MathUtils.rotateVector(
                genericWheelVector,
                heading
            )
        ), Constants.WHEEL_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, swerveWheel, swerveWheel.plus(
            MathUtils.rotateVector(
                MathUtils.rotateVector(
                    genericWheelVector,
                    heading
                ).transpose(),
                driveTrain.getSwivelAngle()
            )
        ), Constants.WHEEL_STROKE_COLOR);



        // calculating the instantaneous center of rotation
        double turnRadius = driveTrain.getTurnRadius();
        SimpleMatrix ICC = new SimpleMatrix(new double[][] {{ -turnRadius, 0 }});
        ICC = MathUtils.rotateVector(ICC, heading).scale(Constants.INCHES_PER_METER);

        // drawing ICC and turn radius
        CanvasUtils.drawDottedLine(fieldOverlay, ICC, position.scale(Constants.INCHES_PER_METER), Constants.TURN_RADIUS_STROKE_COLOR, Constants.DOTTED_LINE_DASH_LENGTH);
        fieldOverlay.strokeCircle(ICC.get(0), ICC.get(1), Math.abs(turnRadius * Constants.INCHES_PER_METER));
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("articulation", articulation);

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

    public boolean articulate(Articulation articulation) {
        this.articulation = articulation;

        switch(articulation) {
            case START:
                if(start.execute()) {
                    this.articulation = Articulation.MANUAL;
                    return true;
                }
                break;
            case MANUAL:
                return true;
            case TRANSFER:
                if(transfer()){
                    this.articulation = Articulation.MANUAL;
                    return true;
                }
                break;
        }
        return false;
    }

    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> {})
                .stateEndAction(() -> {})
                .stage(stage);
    }

    // Misc. Articulations
    private Stage startStage = new Stage();
    private StateMachine start = getStateMachine(startStage)
            .addSingleState(() -> crane.enablePwm())
            .addTimedState(1f, () -> gripper.actuateGripper(false), () -> {})
            .addTimedState(2f, () -> driveTrain.handleDuckSpinner(-0.5), () -> driveTrain.handleDuckSpinner(0))
            .addSingleState(()-> crane.Do(Crane.CommonPosition.HOME))
            .addSingleState(() -> driveTrain.setTargetChassisDistance(Constants.DEFAULT_TARGET_DISTANCE))
            .addSingleState(() -> driveTrain.setMaintainChassisDistanceEnabled(true))
            .build();

    // Tele-Op articulations

    int handOffState = 0;
    double transferTimer = 0.0;
    public boolean transfer(){
        switch(handOffState){
            case 0:
                crane.Do(Crane.CommonPosition.TRANSFER);
                handOffState++;
                break;
            case 1:
                if(crane.isFinished()){
                    gripper.pitchGripper(true);
                    handOffState++;
                }
                break;
            case 2:
                if(gripper.safeToTransfer){
                    gripper.toggleGripper();
                    transferTimer = System.nanoTime();
                    handOffState++;
                }
                break;
            case 3:
                if(transferTimer - System.nanoTime() > 1 / 1E9){
                    gripper.actuateGripper(false);
                    gripper.pitchGripper(false);
                    transferTimer = System.nanoTime();
                    handOffState++;

                }
            case 4:
                if(transferTimer - System.nanoTime() > 1 / 1E9){
                    handOffState = 0;
                    return true;
                }

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
