package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import static org.firstinspires.ftc.teamcode.util.utilMethods.between360Clockwise;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class Turret implements Subsystem {
    private static final String TELEMETRY_NAME = "Turret";
    public static double TURRET_TOLERANCE = 2;

    private DcMotorEx motor;

    private double heading;
    private double targetHeading;
    private double ticksPerDegree = 160.0/90.0;

    public Turret(HardwareMap hardwareMap, boolean simulated) {
        motor = simulated ? new DcMotorExSim(Constants.USE_MOTOR_SMOOTHING) : hardwareMap.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    public void update(Canvas fieldOverlay){
         if(targetHeading > 90)
             targetHeading = 90;
         else if(targetHeading < -90)
             targetHeading = -90;

        heading = motor.getCurrentPosition() / ticksPerDegree;

        motor.setTargetPosition((int)(targetHeading * ticksPerDegree));
    }

    public boolean setTargetAngle(double angle){
        targetHeading = angle;
        return isTurretNearTarget();
    }

    public double getTargetAngle() {
        return targetHeading;
    }

    public double getHeading(){return motor.getCurrentPosition() / ticksPerDegree;}

    public boolean isTurretNearTarget(){
        return between360Clockwise(heading, targetHeading - TURRET_TOLERANCE, heading + TURRET_TOLERANCE);
    }

    private void setPower(double power){
        motor.setPower(power);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        if(debug) {
            telemetryMap.put("turretMotorPosition", motor.getCurrentPosition());
            telemetryMap.put("motor amps", motor.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("turretTargetPos", getTargetAngle());
            telemetryMap.put("turretCurrentAngle", getHeading());
            telemetryMap.put("turretCurrentTest", ticksPerDegree);
        }


        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }
}


