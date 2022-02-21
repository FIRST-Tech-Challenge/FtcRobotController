package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class Turret implements Subsystem {

    private static final double TICKS_PER_DEGREE = 160.0 / 90.0;

    public static double TURRET_TOLERANCE = 2;
    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.003, 0, 0);

    private final PIDController turretPID;
    private final boolean simulated;

    private DcMotorEx motor;

    private double power;
    private double heading, targetHeading;

    public Turret(HardwareMap hardwareMap, boolean simulated) {
        this.simulated = simulated;
        motor = simulated ? new DcMotorExSim(USE_MOTOR_SMOOTHING) : hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretPID = new PIDController(TURRET_PID);
        turretPID.setInputRange(0, 360);
        turretPID.setContinuous(true);
        turretPID.setTolerance(TURRET_TOLERANCE);
        turretPID.enable();
    }

    private double getHeadingCorrection() {
        turretPID.setInput(heading);
        turretPID.setSetpoint(targetHeading);

        return turretPID.performPID();
    }

    public void update(Canvas fieldOverlay){
        if(targetHeading > 90 && targetHeading < 180)
             targetHeading = 90;
         else if(targetHeading > 180 && targetHeading < 270)
             targetHeading = 270;

        heading = simulated ? targetHeading : wrapAngle(motor.getCurrentPosition() / TICKS_PER_DEGREE);

        power = getHeadingCorrection();
        motor.setPower(power);
    }

    public void stop() {
        setTargetHeading(heading);
        power = 0;
    }

    public boolean setTargetHeading(double targetHeading){
        this.targetHeading = wrapAngle(targetHeading);
        return isTurretNearTarget();
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public double getHeading() {
        return heading;
    }

    public boolean isTurretNearTarget(){
        return turretPID.onTarget();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        if(debug) {
            telemetryMap.put("heading", heading);
            telemetryMap.put("target heading", targetHeading);
            telemetryMap.put("motor amps", motor.getCurrent(CurrentUnit.AMPS));
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Turret";
    }
}


