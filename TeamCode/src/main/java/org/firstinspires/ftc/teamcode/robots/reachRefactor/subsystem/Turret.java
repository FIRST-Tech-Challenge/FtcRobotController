package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;
import static org.firstinspires.ftc.teamcode.util.utilMethods.between360Clockwise;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "FFTurret")
public class Turret implements Subsystem {

    public static final double TICKS_PER_DEGREE = 160.0 / 90.0;
    public static double TURRET_TOLERANCE = 10;

    private final boolean simulated;

    private DcMotorEx motor;

    private double power;
    private double heading, targetHeading;

    public Turret(HardwareMap hardwareMap, boolean simulated) {
        this.simulated = simulated;
        motor = simulated ? new DcMotorExSim(USE_MOTOR_SMOOTHING) : hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    public void update(Canvas fieldOverlay) {
        heading = wrapAngle(motor.getCurrentPosition() / TICKS_PER_DEGREE);
        if(heading > 180)
            heading -= 360;

        targetHeading = Range.clip(targetHeading, -90, 90);
        heading = simulated ? targetHeading : Range.clip(heading, -90, 90);

        motor.setTargetPosition((int)((targetHeading * TICKS_PER_DEGREE)));
    }

    public void stop() {
        setTargetHeading(heading);
        power = 0;
    }

    public void setTargetHeading(double targetHeading){
        targetHeading = wrapAngle(targetHeading);
        if(targetHeading > 180)
            targetHeading -= 360;
        this.targetHeading = targetHeading;
        isTurretNearTarget();
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public double getHeading() {
        return heading;
    }

    public boolean isTurretNearTarget(){
        return between360Clockwise(heading, targetHeading - TURRET_TOLERANCE, heading + TURRET_TOLERANCE);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        if(debug) {
            telemetryMap.put("turret heading", heading);
            telemetryMap.put("target turret heading", targetHeading);
            telemetryMap.put("turret motor amps", motor.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("turret near target", isTurretNearTarget());
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Turret";
    }
}


