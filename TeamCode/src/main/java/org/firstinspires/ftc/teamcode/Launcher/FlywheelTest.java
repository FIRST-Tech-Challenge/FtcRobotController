package org.firstinspires.ftc.teamcode.Launcher;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;

import java.io.Writer;

@Config
public class Flywheel {

    public static double kS = 0, kA = 0, kV = 2.6;
    public static double vF = 0.03, vP = 4, vI = 5, vD = 0;
    public static double targetRPM = 1575;
    public static double targetVelocity = 0;
    private final MotorWrapper flywheelmotor;
    Telemetry telemetry;

    public Flywheel(NewIntakeTransferHWmap hwMap, Telemetry telemetry) {
        flywheelmotor = new MotorWrapper(hwMap.getFlywheelMotor(), true, 1, false);
        this.telemetry = telemetry;
    }

    public Flywheel(HWMap hwMap, Telemetry telemetry) {
        flywheelmotor = new MotorWrapper(hwMap.getFlywheelMotor(), true, 1, false);
        this.telemetry = telemetry;
    }

    public void update() {
        flywheelmotor.readVelocity();

        if (targetRPM != 0) {
            targetVelocity = (targetRPM * flywheelmotor.getTicksPerRev()) / 60.0;
            flywheelmotor.setRPM(targetRPM);
        } else if (targetVelocity != 0) {
            targetRPM = (targetVelocity * 60.0) / flywheelmotor.getTicksPerRev();
            flywheelmotor.setVelocity(targetVelocity);
        } else {
            flywheelmotor.setVelocity(0);
        }

        flywheelmotor.setVelocityConstants(vF, vP, vI, vD);
        flywheelmotor.setVelocityConstants(vP, vI, vD, kS, kV, kA);
        telemetry.addData("Current RPM", flywheelmotor.getRPM());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current Velocity (ticks/s)", flywheelmotor.getVelocity());
        telemetry.addData("Target Velocity (ticks/s)", targetVelocity);
        telemetry.addData("Ticks Per Rev", flywheelmotor.getTicksPerRev());
    }

    public MotorWrapper getFlywheelMotor() {
        return flywheelmotor;
    }
}


