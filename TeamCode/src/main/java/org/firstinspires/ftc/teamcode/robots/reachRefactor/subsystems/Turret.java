package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

//written by Cooper Clem, 2021

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.RateController;

import static org.firstinspires.ftc.teamcode.util.utilMethods.between360Clockwise;
import static org.firstinspires.ftc.teamcode.util.utilMethods.diffAngle2;
import static org.firstinspires.ftc.teamcode.util.utilMethods.nextCardinal;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrap360;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrapAngleMinus;

import java.util.HashMap;
import java.util.Map;

@Config
public class Turret implements Subsystem {
    //motor
    private DcMotor motor;
    private double correction;

    //PID
    private PIDController turretPID;

    private double turretHeading;
    private double targetTurretHeading;

    private static final String TELEMETRY_NAME = "Turret";

    public Turret(HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotor.class, "turret");
        turretPID = new PIDController(0,0,0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(){
        turretHeading = motor.getCurrentPosition() / (1 / Constants.TURRET_TICKS_PER_DEGREE);

        movePIDTurret(Constants.TURRET_PID_COEFFICIENTS.p, Constants.TURRET_PID_COEFFICIENTS.i, Constants.TURRET_PID_COEFFICIENTS.d, turretHeading, targetTurretHeading);
    }

    @Override
    public void stop() {

    }

    public boolean rotateCardinalTurret(boolean right){

        setTurretAngle(nextCardinal(getHeading(),right,10));

        return true;
    }

    public boolean setTurretAngle(double angle){
        targetTurretHeading = wrap360(angle);
        return isTurretNearTarget();
    }

    public boolean isTurretNearTarget(){
        return between360Clockwise(getHeading(), getTargetHeading() - Constants.TURRET_TOLERANCE, getTargetHeading() + Constants.TURRET_TOLERANCE);
    }

    private void setPower(double pwr){
        motor.setPower(pwr);
    }

    double turnError = 0;
    public void movePIDTurret(double Kp, double Ki, double Kd, double currentAngle, double targetAngle) {
        //initialization of the PID calculator's output range, target value and multipliers
        turretPID.setOutputRange(-.69, .69); //this is funny
        turretPID.setPID(Kp, Ki, Kd);
        turretPID.setSetpoint(targetAngle);
        turretPID.enable();

        //initialization of the PID calculator's input range and current value
        turretPID.setInputRange(-90, 90);
        turretPID.setInput(currentAngle);

        turnError = diffAngle2(targetAngle, currentAngle);

        double correction = turretPID.performPID();

        //performs the turn with the correction applied
        setPower(correction);
    }

    public double getHeading(){
        return turretHeading;
    }

    public double getTargetHeading(){
        return targetTurretHeading;
    }

    public double getCorrection(){return correction;}
    public double getMotorPwr(){return motor.getPower();}

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<String, Object>();

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }
}


