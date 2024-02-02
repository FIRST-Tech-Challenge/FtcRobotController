package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;
import org.firstinspires.ftc.teamcode.utils.PID.*;

import java.util.function.DoubleSupplier;

public class Arm implements Subsystem {
    private HardwareMap map;
    private AnalogInput potentiometer1;
    private AnalogInput potentiometer2;
    private Telemetry m_telemetry;
    private MotorEx arm1;
    private MotorEx arm2;
    private SimpleServo servo;
    private BTController m_controller;
    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    private ProfiledPIDController m_pid1;
    private ProfiledPIDController m_pid2;

    public Arm(HardwareMap map, Telemetry telemetry, MotorEx arm1, MotorEx arm2) {
        this.map = map;
        this.m_telemetry = telemetry;
        this.arm1 = arm1;
        this.arm2 = arm2;
        potentiometer1 = map.get(AnalogInput.class, "pt1");//port 3
        potentiometer2 = map.get(AnalogInput.class, "pt2");//port 1
        servo = new SimpleServo(map, "armServo", 0, 280);
        register();

    }

    public double AngleToVoltageA1() {
        double ptVoltage = ( desired_first_joint_angle * (vMax1 - vMin1) / (a1Max - arm1Min) + vMin1;
        return ptVoltage;
    }
    public double AngleToVoltageA2() {
        double ptVoltage = ( desired_second_joint_angle * (vMax1 - vMin1) / (a1Max - arm1Min) + vMin1;
        return ptVoltage;
    }

    public void setMotors(double firstSpeed, double secondSpeed, double servoPos) {
        if (potentiometer1.getVoltage() > vMax1 || potentiometer1.getVoltage() < vMin1) {
            arm1.set(0);

        } else {
            arm1.set(firstSpeed);
        }
        if (potentiometer2.getVoltage() > vMax2 || potentiometer2.getVoltage() < vMin2) {
            arm2.set(0);
        } else {
            arm2.set(secondSpeed);

        }
        servo.setPosition(servoPos);

    }


    public BTCommand armMoveManual(DoubleSupplier speedFirst, DoubleSupplier speedSecond, DoubleSupplier posServo) {
        return new RunCommand(() -> {
            arm1.set(speedFirst.getAsDouble());
            setMotors(speedFirst.getAsDouble(), speedSecond.getAsDouble(), posServo.getAsDouble());

        });
    }

    @Override
    public void periodic() {
        dashboard.addData("potent1:", potentiometer1.getVoltage());
        dashboard.addData("potent2:", potentiometer2.getVoltage());
        dashboard.update();
    }
}
