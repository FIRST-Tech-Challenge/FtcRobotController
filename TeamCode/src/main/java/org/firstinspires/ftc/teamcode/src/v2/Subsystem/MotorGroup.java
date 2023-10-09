package org.firstinspires.ftc.teamcode.src.v2.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.src.v2.maths.PIDcontroller;

public class MotorGroup {

    public final DcMotorEx[] motors;
    private final PIDcontroller controller = new PIDcontroller(0, 0, 0, 0, 100);

    public MotorGroup(DcMotorEx... motors) {
        this.motors = motors;
    }

    public void setPIDgains(double Kp, double Kd, double Ki, double Kf, double limit) {
        controller.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

    public void setPowers(double... powers) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    public void setPower(double power, int motor) {
        motors[motor].setPower(power);
    }

    public double getPosition(int motor) {
        return motors[motor].getCurrentPosition();
    }

    //TODO set power of more than one motor
    public void setPositions(double target) {
        setPowers(controller.pidOut(target - getAveragePosition()));
    }

    public double getAveragePosition() {
        double state = 0;

        for (int i = 0; i < motors.length; i++) {
            state += motors[i].getCurrentPosition();
        }
        return state /= motors.length;
    }


}