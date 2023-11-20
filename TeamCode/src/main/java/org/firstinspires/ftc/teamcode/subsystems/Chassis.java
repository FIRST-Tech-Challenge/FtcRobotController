package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class Chassis implements Subsystem {

    private HardwareMap map;
    private Telemetry m_telemetry;
    private MotorEx motor_FL;
    private MotorEx motor_FR;
    private MotorEx motor_BL;
    private MotorEx motor_BR;
    public Chassis(HardwareMap map, Telemetry telemetry){
        this.map=map;
        this.m_telemetry=telemetry;
        motor_FL = new MotorEx(map, "motor_FL");
        motor_FR = new MotorEx(map, "motor_FR");
        motor_BL = new MotorEx(map, "motor_BL");
        motor_BR = new MotorEx(map, "motor_BR");

    }
    public void setMotors (double FL, double FR, double BL, double BR){
        motor_FR.set(FR);
        motor_FL.set(FL);
        motor_BR.set(BR);
        motor_BL.set(BL);
    }


    public Command drive(DoubleSupplier x, DoubleSupplier theta, DoubleSupplier y) {
        return new RunCommand(()->{
            double r = Math.hypot(y.getAsDouble(), theta.getAsDouble());
            double robotAngle = Math.atan2(y.getAsDouble(), theta.getAsDouble()) - Math.PI / 4;//shifts by 90 degrees so that 0 is to the right
            double rightX = x.getAsDouble();
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            setMotors(v1, v2, v3, v4);
        },this);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }



}
