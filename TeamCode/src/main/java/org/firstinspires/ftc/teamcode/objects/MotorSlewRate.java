package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Changes a motor's power a little at a time instead of all at once.
 *
 * <p>Each call moves the power at most {@code maxChangePerLoop} toward the target. Called
 * once a loop, 0.2 takes 5 loops (about 0.1 seconds) to go from stopped to full.
 * This keeps the robot from jerking and tipping when the driver slams the stick.
 */
public class MotorSlewRate {
    /**
     * Move a motor's power toward {@code targetPower}, but by no more than
     * {@code maxChangePerLoop}.
     */
    public static void Set(DcMotor motor, double targetPower, double maxChangePerLoop) {
        double currentPower = motor.getPower();
        double wantedChange = targetPower - currentPower;
        double allowedChange = Math.max(-maxChangePerLoop, Math.min(wantedChange, maxChangePerLoop));
        motor.setPower(currentPower + allowedChange);
    }
}
