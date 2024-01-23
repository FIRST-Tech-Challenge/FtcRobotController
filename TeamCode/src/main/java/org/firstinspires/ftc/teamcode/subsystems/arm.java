package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
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
    private MotorEx motor_first;
    private MotorEx motor_second;
    private SimpleServo servo;
   private BTController m_controller;
    public arm(HardwareMap map, Telemetry telemetry){
        this.map=map;
        this.m_telemetry=telemetry;
        motor_first = new MotorEx(map, "motor_FL");
        motor_second = new MotorEx(map, "motor_FL");
        servo = new SimpleServo(map, "motor_FL", 0, 280) {
        };

    }

    public void setMotors(double firstSpeed, double secondSpeed, double servoPos){
        motor_first.set(firstSpeed);
        motor_second.set(secondSpeed);
        servo.setPosition(servoPos);
    }

    public BTCommand armMove(DoubleSupplier speedFirst,DoubleSupplier speedSecond,DoubleSupplier posServo){
        return new RunCommand(()->{
            motor_first.set(speedFirst.getAsDouble());
            setMotors(speedFirst.getAsDouble(),speedSecond.getAsDouble(),posServo.getAsDouble());
        });
    }

}
