package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;


import android.graphics.Bitmap;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.CanvasUtils;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.MathUtils;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;

public class Robot implements Subsystem {
    // Subsystems
    public DriveTrain driveTrain;
    public Crane crane;
    public Subsystem[] subsystems;

    // State
    private Constants.Alliance alliance;
    private Articulation articulation;

    private static final String TELEMETRY_NAME = "Robot";

    public Robot(HardwareMap hardwareMap) {
        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap);
        crane = new Crane(hardwareMap);
        subsystems = new Subsystem[] {driveTrain, crane};

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
        AUTONOMOUS_BLUE
    }

    public boolean articulate(Articulation articulation) {
        this.articulation = articulation;

        switch(articulation) {
            case MANUAL:
                return true;
        }
        return false;
    }

    // Tele-Op articulations

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
