package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;

import java.util.function.DoubleSupplier;

public class Arm {
    private HardwareMap map;
    private AnalogInput potentiometer;
    private Telemetry m_telemetry;
    private MotorEx arm1;
    private MotorEx arm2;
    private SimpleServo servo;
   private BTController m_controller;
    public Arm(HardwareMap map, Telemetry telemetry, MotorEx arm1, MotorEx arm2, AnalogInput potentiometer) {
        this.map = map;
        this.m_telemetry = telemetry;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.potentiometer=potentiometer;
        servo = new SimpleServo(map, "armServo", 0, 280);
    }

    public void setMotors(double firstSpeed, double secondSpeed, double servoPos){
        if (potentiometer.getVoltage() < Constants.armMax && potentiometer.getVoltage() > Constants.armMin) {
            arm1.set(firstSpeed);
            arm2.set(secondSpeed);
            servo.setPosition(servoPos);
        }else {
            arm1.set(0);
            arm2.set(0);
            servo.setPosition(0);
        }
    }

    public BTCommand armMoveManual(DoubleSupplier speedFirst, DoubleSupplier speedSecond, DoubleSupplier posServo){
        return new RunCommand(()->{
            arm1.set(speedFirst.getAsDouble());
            setMotors(speedFirst.getAsDouble(),speedSecond.getAsDouble(),posServo.getAsDouble());
        });
    }

}
