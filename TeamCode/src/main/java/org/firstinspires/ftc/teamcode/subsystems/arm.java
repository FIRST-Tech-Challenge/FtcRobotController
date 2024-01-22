package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;

import java.util.function.DoubleSupplier;

public class arm {
    private HardwareMap map;
    private Telemetry m_telemetry;
    private MotorEx motor_down;
    private MotorEx motor_up;
    private MotorEx motor_upup;
   private BTController m_controller;
    public arm(HardwareMap map, Telemetry telemetry){
        this.map=map;
        this.m_telemetry=telemetry;
        motor_down= new MotorEx(map, "motor_FL");
        motor_up= new MotorEx(map, "motor_FL");
        motor_upup= new MotorEx(map, "motor_FL");

    }

    public void setMotors(double up, double down, double upup){
        motor_down.set(down);
        motor_up.set(up);
        motor_upup.set(upup);
    }

    public BTCommand downArm(DoubleSupplier speed){
        return new RunCommand(()->{
            motor_down.set(speed.getAsDouble());
        });
    }

    public BTCommand upArm(DoubleSupplier speed){
        return new RunCommand(()->{
            motor_up.set(speed.getAsDouble());
        });
    }

    public BTCommand upupArm(DoubleSupplier speed){
        return new RunCommand(()->{
            motor_upup.set(speed.getAsDouble());
        });
    }
}
