package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class ArmPIDControllerMotionProfile {

    public static double kp=6.5, ki=0, kd=0;
    TrapezoidProfile.Constraints constraintFast = new TrapezoidProfile.Constraints(2800, 3000);
    TrapezoidProfile.Constraints constraintsSlow = new TrapezoidProfile.Constraints(200, 300);
    DcMotorEx armMotor;
    ProfiledPIDController pidController;
    int target;
    private final double ticks_in_degree = 2785 / 360;
    public static double f_arm=75;

    public ArmPIDControllerMotionProfile(DcMotorEx armMotor) {
        this.armMotor = armMotor;

        pidController= new ProfiledPIDController(kp, ki, kd, constraintFast);
        pidController.setTolerance(3);
    }

    public void setTarget(int target) {
        pidController.setGoal(target);
        this.target = target;
    }

    public double calculateVelocity(){
        if (armMotor.getCurrentPosition()>800 && target>900){
            pidController.setConstraints(constraintsSlow);
        } else {
            pidController.setConstraints(constraintFast);
        }

        return pidController.calculate(armMotor.getCurrentPosition())+ Math.cos(Math.toRadians((target+325)/ticks_in_degree))*f_arm;
    }

    public double calculateVelocity(int armPos){
//        pidController.setD(kd);
//        pidController.setP(kp);
//        pidController.setI(ki);
        if (armPos>800 && target>900){
            pidController.setConstraints(constraintsSlow);
        } else {
            pidController.setConstraints(constraintFast);
        }
        //     double velocity = pidController.calculate(pos)+ Math.sin(Math.toRadians((target+325)/ degree_per_ticks))*f_arm;
        if (target ==0 && armPos <50){
            return 0;
        }
        return pidController.calculate(armPos)+ Math.sin(Math.toRadians((target+325)/ degree_per_ticks))*f_arm;
    }

}
