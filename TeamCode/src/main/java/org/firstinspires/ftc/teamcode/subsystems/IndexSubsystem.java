package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

/** M2 - the index that moves balls from the intake up to the shooter. */
public class IndexSubsystem extends SubsystemBase {

    private final DcMotorEx motor;

    public IndexSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.INDEX_NAME);
        motor.setDirection(RobotConstants.INDEX_DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Feeds balls toward the shooter. */
    public void forward() {
        motor.setPower(RobotConstants.INDEX_POWER);
    }

    /** Runs the index backwards - the same speed, negated. Use this to clear a jam. */
    public void reverse() {
        motor.setPower(-RobotConstants.INDEX_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }

    public double getPower() {
        return motor.getPower();
    }
}
