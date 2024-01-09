package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx motor;
    private final double speed = 0.6;  // TODO: Speed Value Might Change

    public IntakeSubsystem(HardwareMap hm) {
        this.motor = new MotorEx(hm, "intake");
        motor.setInverted(true);
    }

    public void run() {
        motor.set(speed);
    }

    public void reverse() {
        motor.set(-speed);
    }

    public void stop() {
        motor.set(0);
    }
}
