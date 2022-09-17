package org.firstinspires.ftc.forteaching;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


// This is just a simple, stupid swerve control system
// It allows you to *either* drive in a particular direction
// or rotate the bot at a given speed
// but *NOT BOTH AT THE SAME TIME*
public class SimpleSwerveDrive {
    private static double FL_RR_ROT_ANGLE = -45.0;
    private static double FR_RL_ROT_ANGLE = 45.0;
    private static int TRANSITION_DELAY_MS = 100;

    private SwerveController ctrl;

    public SimpleSwerveDrive(DcMotorEx flm, DcMotorEx frm, DcMotorEx rlm, DcMotorEx rrm, Servo fls, Servo frs, Servo rls, Servo rrs) {
        ctrl = new SwerveController(flm, frm, rlm, rrm, fls, frs, rls, rrs);
    }

    public void setDirectionAndPowerDegrees(double power, double angle) {
        ctrl.setControlDegrees(power, angle, power, angle, power, angle, power, angle);
    }

    public void setDirectionAndPowerRadians(double power, double angle) {
        ctrl.setControlRadians(power, angle, power, angle, power, angle, power, angle);
    }

    public void setRotationSpeed(double speed) {
        ctrl.setControlDegrees(speed, FL_RR_ROT_ANGLE, speed, FR_RL_ROT_ANGLE, speed, FR_RL_ROT_ANGLE, speed, FL_RR_ROT_ANGLE);
    }

    public void stop() {
        ctrl.stop();
    }
}
