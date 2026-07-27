package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * M3 and M4 - the two shooter wheels. They always run together at the same power, so the whole
 * shooter is treated as one subsystem rather than two.
 */
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        leftMotor  = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER_LEFT_NAME);
        rightMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER_RIGHT_NAME);

        leftMotor.setDirection(RobotConstants.SHOOTER_LEFT_DIRECTION);
        rightMotor.setDirection(RobotConstants.SHOOTER_RIGHT_DIRECTION);

        for (DcMotorEx motor : new DcMotorEx[] {leftMotor, rightMotor}) {
            // FLOAT, not BRAKE - braking a flywheel this heavy every time you let go of the
            // button wears the motors out for no benefit. Let it spin down on its own.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Runs both wheels at the given power, -1.0 to 1.0. */
    public void shootAt(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void shootClose() {
        shootAt(RobotConstants.CLOSE_SHOOT_SPEED);
    }

    public void shootFar() {
        shootAt(RobotConstants.FAR_SHOOT_SPEED);
    }

    public void stop() {
        shootAt(0);
    }

    public double getPower() {
        return leftMotor.getPower();
    }

    /**
     * Current wheel speeds in encoder ticks per second. Only meaningful if the shooter motors
     * have encoder cables plugged in - useful for checking the wheels are up to speed before
     * feeding a ball, and the starting point if you add closed-loop velocity control later.
     */
    public double getLeftVelocity() {
        return leftMotor.getVelocity();
    }

    public double getRightVelocity() {
        return rightMotor.getVelocity();
    }
}
