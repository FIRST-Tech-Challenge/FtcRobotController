package org.firstinspires.ftc.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robot_utilities.Vals;

public class WobbleSystem {

    public Motor wobbleArm;
    public Servo wobbleHand;

    public WobbleSystem(Motor wobbleArm, Servo wobbleHand) {
        this.wobbleArm = wobbleArm;
        this.wobbleHand = wobbleHand;

        this.wobbleArm.setRunMode(Motor.RunMode.PositionControl);
        this.wobbleArm.setPositionCoefficient(Vals.wobble_arm_kp);
        this.wobbleArm.setPositionTolerance(15);
    }

    public void arm_to_pos(int pos) {
        wobbleArm.setTargetPosition(pos);
        if(!wobbleArm.atTargetPosition()) {
            wobbleArm.set(Vals.wobble_arm_velocity);
            return;
        }
        wobbleArm.set(0);
    }

    public void arm_down() {
        this.arm_to_pos(Vals.wobble_arm_down_pos);
    }

    public void arm_up() {
        this.arm_to_pos(Vals.wobble_arm_up_pos);
    }

    public void arm_mid() {
        this.arm_to_pos(Vals.wobble_arm_mid_pos);
    }

}
