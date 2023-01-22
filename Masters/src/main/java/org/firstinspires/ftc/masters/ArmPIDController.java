package org.firstinspires.ftc.masters;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class ArmPIDController {

    PIDController controller;

    public static double p_arm = 0.006, i_arm = 0.001, d_arm = 0.0001;
    public static double f_arm = 0.08;

    protected int target = 0;

    public static double multiplier = 0.6;
    public static double multiplierZero = 0.3;

    private final double ticks = 1425;
    private final double ticks_in_degree = 1425 / 360;

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

        double velocity =0;
        if (target == 0) {
            velocity  = power * multiplierZero * ticks;
        } else {
            velocity = power * multiplier * ticks;
        }

        return velocity;

    }

}
