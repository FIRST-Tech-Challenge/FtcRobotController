package org.firstinspires.ftc.teamcode.TeleOps.MainCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class V4B extends SubsystemBase {

    public enum V4BState {
        RETRACT(1.0),
        EXTEND(0.0);

        public double power;

        V4BState(double power) {
            this.power = power;
        }
    }

    private CRServo servo1;
    private CRServo servo2;

    public V4B(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "CRServoL"); // Left Side
        servo2 = hardwareMap.get(CRServo.class, "CRServoR"); // Right Side

        // Set the initial servo powers
        forceDown(); // Puts down at the start
    }

    @Override
    public void periodic() {}

    public double getPower() {
        return servo1.getPower();
    }

    public void forceDown() {
        servo1.setPower(V4BState.RETRACT.power);
        servo2.setPower(V4BState.RETRACT.power);
    }

    public void setPower(V4BState state) {
        servo1.setPower(state.power);
        servo2.setPower(state.power);
    }

    public void togglePower() {
        if (getPower() == V4BState.RETRACT.power) {
            setPower(V4BState.EXTEND);
        } else {
            setPower(V4BState.RETRACT);
        }
    }
}
