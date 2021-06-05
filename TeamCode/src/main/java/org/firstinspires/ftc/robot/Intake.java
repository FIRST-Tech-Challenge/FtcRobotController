package org.firstinspires.ftc.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robot_utilities.Vals;

public class Intake {

    public Motor intake1, intake2;


    public Intake(Motor intake1, Motor intake2) {
        this.intake1 = intake1;
        this.intake2 = intake2;

        this.intake1.setRunMode(Motor.RunMode.VelocityControl);
        this.intake2.setRunMode(Motor.RunMode.VelocityControl);
        this.intake1.setVeloCoefficients(0.05, 0, 0);
        this.intake2.setVeloCoefficients(0.05, 0, 0);
    }

    public void intake_in() {
        intake1.set(Vals.intake_speed);
        intake2.set(Vals.intake_speed);
    }

    public void intake_out() {
        intake1.set(-Vals.intake_speed);
        intake2.set(-Vals.intake_speed);
    }

    public void intake_stop() {
        intake1.set(0);
        intake2.set(0);
    }




}
