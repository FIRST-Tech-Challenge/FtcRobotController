package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Intake motor.
 * Same hardware and directions as QuantumStormTeleOp.
 */
public class Intake {

    private final DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR);

        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    public void start() {
        setPower(Constants.INTAKE_POWER);
    }

    public void stop() {
        setPower(0);
    }

    public void setPower(double power) {
        power=Math.max(-1,Math.min(1,power));
        intake.setPower(power);
    }
}
