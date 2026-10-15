package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.Mechanism;

public final class IntakeController implements Mechanism {
    private DcMotorEx intakeMotor;
    private boolean on;

    public void initialize(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        turnOff();
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
        on = Math.abs(power) > 0.0;
    }

    public void turnOn(boolean reversed) {
        setPower(reversed ? -Constants.INTAKE_POWER : Constants.INTAKE_POWER);
    }

    public void turnOff() {
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        on = false;
    }

    public void toggle(boolean reversed) {
        if (on) {
            turnOff();
        } else {
            turnOn(reversed);
        }
    }

    public boolean isOn() {
        return on;
    }

    public void telemetry(Telemetry telemetry) {
        if (intakeMotor == null) {
            return;
        }
        telemetry.addLine("--- INTAKE ---");
        telemetry.addData("Power", "%.2f", intakeMotor.getPower());
    }
}
