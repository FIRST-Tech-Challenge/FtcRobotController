package org.firstinspires.ftc.masters;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class ArmPIDController {

    PIDController controller;

    public static double p_arm = 0.008, i_arm = 0.00, d_arm = 0.000;
    public static double f_arm = 0.09;

    protected int target = 0;

    public static double multiplier = 0.6;
    public static double multiplierZero = 0.3;

    private final double degree_per_ticks = 360/2786.2 ;

    DcMotorEx armMotor;

    public ArmPIDController(DcMotorEx armMotor) {

        this.armMotor = armMotor;
        controller = new PIDController(p_arm, i_arm, d_arm);



    }

    public void setTarget(int target){
        this.target = target;
    }

    public double calculateVelocity() {
        controller = new PIDController(p_arm, i_arm, d_arm);


        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        double ff = Math.sin(Math.toRadians((target+325)/ degree_per_ticks)) * f_arm;

        double power = pid + ff;


        return power;

    }

    public double calculateVelocity(int armPos) {
        controller = new PIDController(p_arm, i_arm, d_arm);

        double pid = controller.calculate(armPos, target);

        double ff = Math.sin(Math.toRadians((target+325)/ degree_per_ticks)) * f_arm;

        double power = pid + ff;

        if (target > 1100 && armPos>1000){
            if (power>0){
                power = Math.min(power, 0.3);
            } else {
                power = Math.max(power, -0.3);
            }
        }


        return power;

    }


}
