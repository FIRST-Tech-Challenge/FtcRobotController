package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

//written by Cooper Clem, 2021

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.util.utilMethods.between360Clockwise;
import static org.firstinspires.ftc.teamcode.util.utilMethods.diffAngle2;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrap360;

import java.util.HashMap;
import java.util.Map;

@Config
public class Turret implements Subsystem {
    // Motors
    private DcMotorEx motor;

    // PID
    private double correction;
    private PIDController turretPID;

    private double heading;
    private double targetHeading;

    // Constants
    private static final String TELEMETRY_NAME = "Turret";

    public static PIDCoefficients TURRET_PID_COEFFICIENTS = new PIDCoefficients(0.02, 0, 0);
    public static double TURRET_TOLERANCE = 2;

    public static double TICKS_PER_REVOLUTION = 1740;


    public Turret(HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotorEx.class, "turret");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretPID = new PIDController(TURRET_PID_COEFFICIENTS);
    }

    public void update(){
        heading = (motor.getCurrentPosition() / TICKS_PER_REVOLUTION * 2 * Math.PI) % (2 * Math.PI);;


        movePIDTurret(TURRET_PID_COEFFICIENTS, heading, targetHeading);
    }

    @Override
    public void stop() {

    }

    public boolean setTargetAngle(double angle){
        targetHeading = Math.toDegrees(wrap360(angle));
        return isTurretNearTarget();
    }

    public boolean isTurretNearTarget(){
        return between360Clockwise(heading, targetHeading - TURRET_TOLERANCE, heading + TURRET_TOLERANCE);
    }

    private void setPower(double power){
        motor.setPower(power);
    }

    double turnError = 0;
    public void movePIDTurret(PIDCoefficients pidCoefficients, double currentAngle, double targetAngle) {
        //initialization of the PID calculator's output range, target value and multipliers
        turretPID.setPID(pidCoefficients);
        turretPID.setOutputRange(-.69, .69); //this is funny
        turretPID.setSetpoint(targetAngle);
        turretPID.enable();

        //initialization of the PID calculator's input range and current value
        turretPID.setInputRange(-90, 90);
        turretPID.setInput(currentAngle);

        turnError = diffAngle2(targetAngle, currentAngle);

        correction = turretPID.performPID();

        //performs the turn with the correction applied
        setPower(correction);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<String, Object>();
        if(debug) {
            telemetryMap.put("motor position", motor.getCurrentPosition());
            telemetryMap.put("motor power", correction);
            telemetryMap.put("motor amps", motor.getCurrent(CurrentUnit.AMPS));
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }
}


