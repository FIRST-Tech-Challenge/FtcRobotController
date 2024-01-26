package org.firstinspires.ftc.teamcode.subsystems;


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

import java.util.function.DoubleSupplier;

public class Arm {
    private Translation2d point
    private Translation2d current_desired_point;
    private double desired_second_joint_angle
    private double desired_first_joint_angle
    private HardwareMap map;
    private Telemetry m_telemetry;
    private MotorEx motor_first;
    private MotorEx motor_second;
    private SimpleServo servo;
   private BTController m_controller;
    public Arm(HardwareMap map, Telemetry telemetry){
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
    private double calculateFeedForwardFirstJoint(double first_joint_angle){

        return  (resistance * (
                first_arm_weight * (g * l1 * Util.cosInDegrees(first_joint_angle) + robotXAcc * Util.sinInDegrees(first_joint_angle))
        )
                /(gear_ratio * neo_Kt))/ 10;
        // in volts
    }
    private double calculateFeedForwardSecondJoint(double second_joint_angle){
        return (resistance *
                (second_arm_weight * (g * l2 * Util.cosInDegrees(second_joint_angle) + robotXAcc * Util.sinInDegrees(second_joint_angle))
                )
                /(gear_ratio * neo_Kt))/motorMaxVolt;
        //in volts
        // need to convert to pwm
    }
    //    voltage_90_degrees = resistance_motor*torque_90_degrees/(gear_ratio*Kt)
    //calculates the feedForward to second joint using a torque calculation to the current angle
    private void setDesiredAnglesToJointsPositiveX(){
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
                        - Math.toDegrees(
                        Math.atan2(
                                l2 * Util.sinInDegrees(desired_second_joint_angle)
                                , (l1 + l2 * Util.cosInDegrees(desired_second_joint_angle))
                        )
                );
        desired_second_joint_angle = desired_first_joint_angle + desired_second_joint_angle;

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
                                setDesiredAnglesToJointsPositiveX();}
                        }
                    }
                }
}
