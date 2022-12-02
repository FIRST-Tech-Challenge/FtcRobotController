package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {
    private final Motor motor;

    public ElevatorSubsystem(final HardwareMap hardwareMap, final String name) {
        int cpr = 0;
        int rpm =0;

        motor = new Motor(hardwareMap,"",cpr,rpm);
    }

    /**
     * Raise
     */
    public void raise() {
        //TODO
        motor.setTargetDistance(1);
    }

    /**
     * Lower
     */
    public void lower() {
        motor.setTargetDistance(0);
    }
}
