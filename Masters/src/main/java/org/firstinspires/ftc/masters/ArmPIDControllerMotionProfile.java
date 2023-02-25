package org.firstinspires.ftc.masters;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmPIDControllerMotionProfile {

    public static double kp=6, ki=0, kd=0.09;
    TrapezoidProfile.Constraints constraintFast = new TrapezoidProfile.Constraints(2800, 3000);
    TrapezoidProfile.Constraints constraintsSlow = new TrapezoidProfile.Constraints(200, 3000);
    DcMotorEx armMotor;
    ProfiledPIDController pidController;
    int target;

    public ArmPIDControllerMotionProfile(DcMotorEx armMotor) {
        this.armMotor = armMotor;

        pidController= new ProfiledPIDController(kp, ki, kd, constraintFast);
        pidController.setTolerance(3);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public double calculateVelocity(){
        if (armMotor.getCurrentPosition()>800 && target>900){
            pidController.setConstraints(constraintsSlow);
        } else {
            pidController.setConstraints(constraintFast);
        }

        return pidController.calculate(armMotor.getCurrentPosition());
    }
}
