//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.arcrobotics.ftclib.command.Subsystem;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import static org.firstinspires.ftc.teamcode.Constants.*;
//
//public class ArmSubsystem extends Subsystem {
//
//    private
//    private final HardwareMap map;
//    MotorEx arm1;
//    MotorEx arm2;
//    public ArmSubsystem(HardwareMap map){
//        this.map=map;
//        arm1= new MotorEx(map,"arm1");
//        arm2= new MotorEx(map,"arm2");
//
//
//    }
//    private double calculateFeedForwardFirstJoint(double first_joint_angle){
//
//        return  (resistance * (
//                first_arm_weight * (g * l1ff * Util.cosInDegrees(first_joint_angle) + robotXAcc * Util.sinInDegrees(first_joint_angle))
//        )
//                /(gear_ratio * neo_Kt))/ 10;
//        // in volts
//    }
//    private double calculateFeedForwardSecondJoint(double second_joint_angle){
//        return (resistance *
//                (second_arm_weight * (g * l2ff * Util.cosInDegrees(second_joint_angle) + robotXAcc * Util.sinInDegrees(second_joint_angle))
//                )
//                /(gear_ratio * neo_Kt))/motorMaxVolt;
//        //in volts
//        // need to convert to pwm
//    }
//    //    voltage_90_degrees = resistance_motor*torque_90_degrees/(gear_ratio*Kt)
//    //calculates the feedForward to second joint using a torque calculation to the current angle
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
    }

}