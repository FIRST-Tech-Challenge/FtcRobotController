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

    private final double ticks_per_seconds = 2785 * 117 /60;
    private final double ticks_in_degree = 2785 / 360;

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

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f_arm;

        double power = pid + ff;

//        if (power>1){
//            power =1;
//        }
//
//        if (target == 0 && armPos< 400){
//            power = - Math.min(Math.abs(power), 0.2);
//        }

        return power;

    }

}
