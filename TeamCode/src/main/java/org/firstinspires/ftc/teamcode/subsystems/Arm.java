package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.ArmPID.*;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

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
    private PIDController m_pid1;
    private PIDController m_pid2;
    private double desired_first_joint_angle=0;
    private double desired_second_joint_angle=0;
    private Translation2d current_desired_point;
    private double current_second_joint_angle;
    private double current_first_joint_angle;
    private double current_pot1_voltage;
    private double current_pot2_voltage;
    private double arm1PID,arm1FF;
    private double arm2PID,arm2FF;


    public Arm(HardwareMap map, Telemetry telemetry, MotorEx arm1, MotorEx arm2) {
        this.map = map;
        this.m_telemetry = telemetry;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.arm1.setInverted(true);
        this.arm2.setInverted(true);
        m_pid1 = new PIDController(a1KP,a1KI,a1KD);
        m_pid2 = new PIDController(a2KP,a2KI,a2KD);
        potentiometer1 = map.get(AnalogInput.class, "pt1");//port 3
        potentiometer2 = map.get(AnalogInput.class, "pt2");//port 1
        servo = new SimpleServo(map, "armServo", 0, 280);
        register();

    }
    public void setMotorFromAngle1(){
        arm1PID= m_pid1.calculate(current_first_joint_angle,desired_second_joint_angle);
        arm1FF=calculateFeedForwardFirstJoint(current_first_joint_angle);
        dashboard.addData("first joint output:", arm1PID);
        arm1.set(c_arm1FF.calculate(arm1FF+arm1PID));

    }
    public void setMotorFromAngle2(){
        arm2PID= m_pid2.calculate(current_second_joint_angle,desired_second_joint_angle);
        arm2FF=calculateFeedForwardSecondJoint(current_second_joint_angle);
        dashboard.addData("second joint output:", arm2PID);
        arm2.set(c_arm2FF.calculate(arm2FF+arm2PID));

    }

    public double angleToVoltageA1(double angle) {
        double ptVoltage = ( angle-arm1Min) * (vMax1 - vMin1) / (a1Max - arm1Min) + vMin1;
        return ptVoltage;
    }
    public double voltageToAngle1( double voltage) {
        double angle1 = (voltage-vMin1)*(a1Max-arm1Min)/(vMax1-vMin1) + arm1Min;
        return angle1;
    }
    public double angleToVoltageA2(double angle) {
        double ptVoltage = (angle-arm2Min) * (vMax1 - vMin1) / (a1Max - arm1Min) + vMin1;
        return ptVoltage;

    }
    public double voltageToAngle2( double voltage) {
        double angle2 = (voltage-vMin2)*(a2Max-arm2Min)/(vMax2-vMin2)  + arm2Min;
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
        current_first_joint_angle = voltageToAngle1(potentiometer1.getVoltage());
        current_second_joint_angle = voltageToAngle2(potentiometer2.getVoltage());
        current_pot1_voltage = potentiometer1.getVoltage();
        current_pot2_voltage = potentiometer2.getVoltage();
        setMotorFromAngle2();
        setMotorFromAngle1();
        dashboard.addData("desired angle 1:", desired_first_joint_angle);
        dashboard.addData("desired angle 2:", desired_second_joint_angle);
        dashboard.addData("pot1:", current_pot1_voltage);
        dashboard.addData("pot2:", current_pot2_voltage);
        dashboard.addData("first angle ", current_first_joint_angle);
        dashboard.addData("second angle", current_second_joint_angle);
        dashboard.addData("arm1PID", arm1PID);
        dashboard.addData("arm2PID", arm2PID);
        dashboard.addData("FFArm1:", calculateFeedForwardFirstJoint(current_first_joint_angle));
        dashboard.addData("FFArm2:", calculateFeedForwardSecondJoint(current_second_joint_angle));
        dashboard.addData("ArmX", anglesToPoint(current_first_joint_angle,current_second_joint_angle).getX());
        dashboard.addData("ArmY", anglesToPoint(current_first_joint_angle,current_second_joint_angle).getY());
        dashboard.update();

        m_pid1.setPID(a1KP,a1KI,a1KD);
        m_pid2.setPID(a2KP,a2KI,a2KD);
    }

    private double calculateFeedForwardFirstJoint(double first_joint_angle){

        return  ((resistance *
                (first_arm_weight * (g * l1 * Util.cosInDegrees(first_joint_angle) )
                )
                /(first_gear_ratio * neo_Kt))/ motorMaxVolt)/ffConv;//to conv between
        // in volts
    }
    private double calculateFeedForwardSecondJoint(double second_joint_angle){
        return ((resistance *
                (second_arm_weight * (g * l2 * Util.cosInDegrees(second_joint_angle+90))
                )
                /(second_gear_ratio * neo_Kt))/motorMaxVolt)/ffConv;
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
    private void setDesiredAnglesToJointsNegativeX(){
        desired_second_joint_angle = - Util.aCosInDegrees(
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
                        + Math.toDegrees(
                        Math.atan2(
                                l2 * Util.sinInDegrees(desired_second_joint_angle)
                                , (l1 + l2 * Util.cosInDegrees(desired_second_joint_angle))
                        )
                );
        desired_second_joint_angle = desired_first_joint_angle - desired_second_joint_angle;
    }
    public Translation2d anglesToPoint(double firstAngle, double second) {
        double x = Util.cosInDegrees(firstAngle) * l1 + Util.cosInDegrees(180-second+firstAngle) * l2;
        double y = Util.sinInDegrees(firstAngle) * l1 + Util.sinInDegrees(180-second+firstAngle) * l2;
        y = ((int) (y * 1000)) / 1000.0;
        x = ((int) (x * 1000)) / 1000.0;
        return new Translation2d(x, y);
    }


        public void setDesiredPoint(Translation2d point){
            //checks if the given point is already the desired point
            if (!point.equals(current_desired_point)) {
                // checks if the given point is in bounds of possibly
                //checks if point is not in the ground
                if (point.getY() > -0.05) {
                    //checks if point is in arm radius
                    if (Math.hypot(point.getX(), point.getY()) < l1 + l2) {
                        current_desired_point = point;
                        //set angles accordingly -x and x, not the same calculation for both
                        if (current_desired_point.getX() >= 0) {
                            setDesiredAnglesToJointsPositiveX();
                        }
                        else{
                            setDesiredAnglesToJointsNegativeX();
                        }
                    }
                }
            }

        }

        public BTCommand armMoveToPoint(Translation2d point){
        return new RunCommand(
                ()->{setDesiredPoint(point);}
        );
    }

}



