package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.CanvasUtils;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.MathUtils;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.HashMap;
import java.util.Map;

public class Robot implements Subsystem {
    // Telemetry
    private FtcDashboard dashboard;
    private Telemetry telemetry;

    // Subsystems
    public DriveTrain driveTrain;
    private Subsystem[] subsystems;

    // State
    private Constants.Alliance alliance;
    private boolean dashboardEnabled, telemetryDebugEnabled;
    private Map<String, Object> telemetryMap;

    public static final String TELEMETRY_NAME = "Robot";

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean dashboardEnabled) {
        this.telemetry = telemetry;

        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap);
        subsystems = new Subsystem[] {driveTrain};

        telemetryMap = new HashMap<>();

        this.dashboardEnabled = dashboardEnabled;
        if(dashboardEnabled)
            dashboard = FtcDashboard.getInstance();
    }

    private void drawFieldOverlay(TelemetryPacket packet) {
        Canvas fieldOverlay = packet.fieldOverlay();

        SimpleMatrix pose = driveTrain.getPose();

        SimpleMatrix position = pose.rows(0, 2);
        double heading = pose.get(2);

        // calculating wheel positions
        SimpleMatrix leftWheel = new SimpleMatrix(new double[][] {{ -Constants.TRACK_WIDTH / 2 , 0 }});
        SimpleMatrix rightWheel = new SimpleMatrix(new double[][] {{ Constants.TRACK_WIDTH / 2, 0 }});
        SimpleMatrix swerveWheel = new SimpleMatrix(new double[][] {{ 0, -driveTrain.getChassisDistance() }});

        SimpleMatrix rotationMatrix = new SimpleMatrix(new double[][] {
                {Math.cos(heading), -Math.sin(heading)},
                {Math.sin(heading), Math.cos(heading)}
        });
        leftWheel = position.plus(rotationMatrix.mult(leftWheel.transpose())).scale(Constants.INCHES_PER_METER);
        rightWheel = position.plus(rotationMatrix.mult(rightWheel.transpose())).scale(Constants.INCHES_PER_METER);
        swerveWheel = position.plus(rotationMatrix.mult(swerveWheel.transpose())).scale(Constants.INCHES_PER_METER);

        // drawing axles
        CanvasUtils.drawLine(fieldOverlay, leftWheel, rightWheel, Constants.AXLE_STROKE_COLOR);
        CanvasUtils.drawLine(fieldOverlay, leftWheel.plus(rightWheel).divide(2), swerveWheel, Constants.AXLE_STROKE_COLOR);

        // drawing wheel vectors
        CanvasUtils.drawVector(fieldOverlay, leftWheel, Constants.WHEEL_RADIUS * 2 * Constants.INCHES_PER_METER, Math.PI / 2, Constants.WHEEL_STROKE_COLOR);

        // calculating the instantaneous center of rotation
        double turnRadius = driveTrain.getTurnRadius();
        SimpleMatrix ICC = new SimpleMatrix(new double[][] {{ -turnRadius, 0 }});
        ICC = MathUtils.rotateVector(ICC, heading).scale(Constants.INCHES_PER_METER);

        // drawing ICC and turn radius
        CanvasUtils.drawDottedLine(fieldOverlay, ICC, pose.scale(Constants.INCHES_PER_METER), Constants.TURN_RADIUS_STROKE_COLOR, Constants.DOTTED_LINE_DASH_LENGTH);
        fieldOverlay.strokeCircle(ICC.get(0), ICC.get(1), turnRadius * Constants.INCHES_PER_METER);
    }

    public void sendTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();

        // sending additional (opmode) telemetry
        packet.addLine(Constants.DEFAULT_TELEMETRY_LINE);
        packet.putAll(telemetryMap);
        packet.addLine("");

        telemetry.addLine(Constants.DEFAULT_TELEMETRY_LINE);
        for(Map.Entry<String, Object> entry: telemetryMap.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
        telemetry.addLine();

        // sending telemetry for subsystems
        for(Subsystem subsystem: subsystems) {
            Map<String, Object> telemetryMap = subsystem.getTelemetry(telemetryDebugEnabled);
            String telemetryName = subsystem.getTelemetryName();

            packet.addLine(telemetryName);
            packet.putAll(telemetryMap);
            packet.addLine("");

            telemetry.addLine(telemetryName);
            for(Map.Entry<String, Object> entry: telemetryMap.entrySet()) {
                telemetry.addData(entry.getKey(), entry.getValue());
            }
            telemetry.addLine();
        }

        // sending telemetry for robot
        Map<String, Object> robotTelemetry = getTelemetry(telemetryDebugEnabled);
        String telemetryName = getTelemetryName();

        packet.addLine(telemetryName);
        packet.putAll(robotTelemetry);
        packet.addLine("");

        telemetry.addLine(telemetryName);
        for(Map.Entry<String, Object> entry: robotTelemetry.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
        telemetry.addLine();

        // dashboard telemetry
        if(dashboardEnabled) {
            drawFieldOverlay(packet);
            dashboard.sendTelemetryPacket(packet);
        }

        telemetry.update();
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

        sendTelemetry();
    }

    @Override
    public void reset() {
        for(Subsystem subsystem: subsystems)
            subsystem.reset();
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

        // tele-op articulations

        // autonomous articulations
        AUTONOMOUS_RED,
        AUTONOMOUS_BLUE;
    }
    private Articulation articulation;

    public boolean articulate(Articulation articulation) {
        this.articulation = articulation;

        switch(articulation) {
            case MANUAL:
                return true;
            case AUTONOMOUS_BLUE:
                if(autonomousBlue.execute())
                    return true;
            case AUTONOMOUS_RED:
                if(autonomousRed.execute())
                    return true;
        }
        return false;
    }

    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> {})
                .stateEndAction(() -> { articulation = Robot.Articulation.MANUAL; })
                .stage(stage);
    }

    // Tele-Op articulations


    // Autonomous articulations
    private Stage autonomousRedStage = new Stage();
    public StateMachine autonomousRed = getStateMachine(autonomousRedStage)
            // TODO: insert autonomous red states here
            .build();

    private Stage autonomousBlueStage = new Stage();
    public StateMachine autonomousBlue = getStateMachine(autonomousBlueStage)
            // TODO: insert autonomous blue states here
            .build();

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

    public Constants.Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Constants.Alliance alliance) {
        this.alliance = alliance;
    }

    public void toggleIsDashboardEnabled() {
        dashboardEnabled = !dashboardEnabled;
        if(dashboard == null)
            dashboard = FtcDashboard.getInstance();
    }

    public void toggleIsTelemetryDebugEnabled() {
        telemetryDebugEnabled = !telemetryDebugEnabled;
    }

    public void addTelemetryData(String name, Object value) {
        telemetryMap.put(name, value);
    }

    public boolean isDashboardEnabled() {
        return dashboardEnabled;
    }

    public boolean isTelemetryDebugEnabled() { return telemetryDebugEnabled; }
}
