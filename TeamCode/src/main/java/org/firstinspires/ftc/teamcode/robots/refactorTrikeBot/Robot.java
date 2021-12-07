package org.firstinspires.ftc.teamcode.robots.refactorTrikeBot;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

import static org.firstinspires.ftc.teamcode.util.utilMethods.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrapAngleMinus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.utils.CanvasUtils;
import org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.utils.MathUtil;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Robot implements Subsystem {
    // Telemetry
    private FtcDashboard dashboard;
    private Telemetry telemetry;

    // Subsystems
    private DriveTrain driveTrain;
    private Subsystem[] subsystems;

    // Articulations
    public enum Articulation {
        manual
    }
    private Articulation articulation;

    public static final String TELEMETRY_NAME = "Robot";

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap);
        subsystems = new Subsystem[] {driveTrain};

        dashboard = FtcDashboard.getInstance();
    }

    public void sendTelemetry() {
        drawFieldOverlay();

        TelemetryPacket packet = new TelemetryPacket();

        // sending telemetry for subsystems
        for(Subsystem subsystem: subsystems) {
            Map<String, Object> telemetryMap = subsystem.getTelemetry();
            packet.putAll(telemetryMap);

            telemetry.addLine(subsystem.getTelemetryName());
            for(Map.Entry<String, Object> entry: telemetryMap.entrySet()) {
                telemetry.addData(entry.getKey(), entry.getValue());
            }
        }

        // sending telemetry for robot
        Map<String, Object> robotTelemetry = getTelemetry();
        packet.putAll(robotTelemetry);
        telemetry.addLine(getTelemetryName());
        for(Map.Entry<String, Object> entry: robotTelemetry.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }

    @Override
    public Map<String, Object> getTelemetry() {
        Map<String, Object> telemetryMap = new HashMap<String, Object>();
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

    private void drawFieldOverlay() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        SimpleMatrix pose = driveTrain.getPose();

        SimpleMatrix position = pose.cols(0, 1);
        double heading = pose.get(2);

        SimpleMatrix leftWheel = new SimpleMatrix(new double[][] {{ -Constants.TRACK_WIDTH / 2 }, { 0 }});
        SimpleMatrix rightWheel = new SimpleMatrix(new double[][] {{ Constants.TRACK_WIDTH / 2 }, { 0 }});
        SimpleMatrix swerveWheel = new SimpleMatrix(new double[][] {{ 0 }, { -driveTrain.getChassisDistance() }});

        SimpleMatrix rotationMatrix = new SimpleMatrix(new double[][] {
                {Math.cos(heading), -Math.sin(heading)},
                {Math.sin(heading), Math.cos(heading)}
        });
        leftWheel = position.plus(rotationMatrix.mult(leftWheel));
        rightWheel = position.plus(rotationMatrix.mult(rightWheel));
        swerveWheel = position.plus(rotationMatrix.mult(swerveWheel));

        CanvasUtils.drawLine(fieldOverlay, leftWheel, rightWheel, "#000000");
        CanvasUtils.drawLine(fieldOverlay, leftWheel.plus(rightWheel).divide(2), swerveWheel, "#4D934D");
    }
}
