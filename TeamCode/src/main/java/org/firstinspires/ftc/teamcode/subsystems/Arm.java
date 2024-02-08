package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.*;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.BTController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;
import org.firstinspires.ftc.teamcode.utils.Util;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;

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
    private double desired_first_joint_angle;
    private double desired_second_joint_angle;
    private Translation2d current_desired_point;


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
    public void setMotorFromAngle1(){
        double voltageSetPoint = angleToVoltageA1();
        double currentVoltageMeasure=potentiometer1.getVoltage();
        double currentVoltageMeasure2 =potentiometer2.getVoltage();
//        double PIDvalue= m_pid1.calculate(voltageSetPoint,currentVoltageMeasure);
        double motorSet = calculateFeedForwardFirstJoint(voltageToAngle1(currentVoltageMeasure))
                +calculateFeedForwardSecondJoint(voltageToAngle2(currentVoltageMeasure2));
        dashboard.addData("first joint output:", motorSet);
//        arm1.set(motorSet);
    }
    public void setMotorFromAngle2(){
        double voltageSetPoint = angleToVoltageA2();
        double currentVoltageMeasure=potentiometer2.getVoltage();
//        double PIDvalue= m_pid2.calculate(voltageSetPoint,currentVoltageMeasure);
        double motorSet = calculateFeedForwardSecondJoint(voltageToAngle2(currentVoltageMeasure));
        dashboard.addData("second joint output:", motorSet);
//        arm2.set(motorSet);

    }


    public double angleToVoltageA1() {
        double ptVoltage = ( desired_first_joint_angle * (vMax1 - vMin1) / (a1Max - arm1Min) + vMin1);
        return ptVoltage;
    }
    public double voltageToAngle1( double voltage) {
        double angle1 = (voltage-vMin1)*(a1Max-arm1Min)/vMax1-vMin1;
        return angle1;
    }
    public double angleToVoltageA2() {
        double ptVoltage = ( desired_second_joint_angle * (vMax1 - vMin1) / (a1Max - arm1Min) + vMin1);
        return ptVoltage;

    }
    public double voltageToAngle2( double voltage) {
        double angle2 = (voltage-vMin2)*(a2Max-arm2Min)/vMax2-vMin2;
        return angle2;
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
            arm2.set(speedSecond.getAsDouble());

        });
    }

    @Override
    public void periodic() {
        dashboard.addData("potent1:", potentiometer1.getVoltage());
        dashboard.addData("potent2:", potentiometer2.getVoltage());
        setMotorFromAngle2();
        setMotorFromAngle1();
        dashboard.update();
    }

    private double calculateFeedForwardFirstJoint(double first_joint_angle){

        return  (resistance * (
                first_arm_weight * (g * l1 * Util.cosInDegrees(first_joint_angle) )
        )
                /(first_gear_ratio * neo_Kt))/ 10;
        // in volts
    }
    private double calculateFeedForwardSecondJoint(double second_joint_angle){
        return (resistance *
                (second_arm_weight * (g * l2 * Util.cosInDegrees(second_joint_angle))
                )
                /(second_gear_ratio * neo_Kt))/motorMaxVolt;
        //in volts
        // need to convert to pwm
    }
    //    voltage_90_degrees = resistance_motor*torque_90_degrees/(gear_ratio*Kt)
    //calculates the feedForward to second joint using a torque calculation to the current angle
    private void setDesiredAnglesToJointsPositiveX() {
        desired_second_joint_angle = -Util.aCosInDegrees(
                (Math.pow(current_desired_point.getX(), 2)
                        + Math.pow(current_desired_point.getY(), 2)
                        - Math.pow(l1, 2) - Math.pow(l2, 2))
                        / (2 * l1 * l2));
        desired_first_joint_angle =
                Math.toDegrees(Math.atan2(
                                current_desired_point.getY(),
                                current_desired_point.getX()
                        )

                )
                        - Math.toDegrees(
                        Math.atan2(
                                l2 * Util.sinInDegrees(desired_second_joint_angle)
                                , (l1 + l2 * Util.cosInDegrees(desired_second_joint_angle))
                        )
                );
        desired_second_joint_angle = desired_first_joint_angle + desired_second_joint_angle;
    }
        public void setDesiredPoint(Translation2d point){
            //checks if the given point is already the desired point
            if (!point.equals(current_desired_point)) {
                // checks if the given point is in bounds of possibly
                //checks if point is not in the ground
                if (point.getY() > -0.09) {
                    //checks if point is in arm radius
                    if (Math.hypot(point.getX(), point.getY()) < l1 + l2) {
                        current_desired_point = point;
                        //set angles accordingly -x and x, not the same calculation for both
                        if (current_desired_point.getX() >= 0) {
                            setDesiredAnglesToJointsPositiveX();
                        }
                    }
                }
            }

        }
    }

