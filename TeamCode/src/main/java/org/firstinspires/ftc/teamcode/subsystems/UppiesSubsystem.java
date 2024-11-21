package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.BooleanSupplier;

public class UppiesSubsystem extends SubsystemBase {

    DcMotorEx uppiesMotor;
    private UppiesState state;
    long lastStateChange = 0;

    public static final boolean PROGRAMATIC_STALL_SAFETY = true;
    // After the motor has enough time to start moving, start checking if the motor is stalling.
    static final long STALL_THRESHOLD = 300;
    // Once the motor is moving less than the specified ticks per second, it is stalling.
    public static final double STALL_TICKS_PER_SECOND_THRESHOLD = 2;

    public UppiesSubsystem(DcMotorEx uppiesMotor) {
        this.uppiesMotor = uppiesMotor;
        this.state = UppiesState.IDLE;
    }

    public void setUppiesState(UppiesState state) {
        UppiesState lastState = state;
        this.state = state;
        // TODO: Verify that this is the correct direction
        uppiesMotor.setPower(state == UppiesState.UN_UPPIES ? 1 : -1);
        if (state == UppiesState.IDLE) {
            uppiesMotor.setPower(0);
        }
        // If the state changed, reset the timer. Allows for this method to be called periodically/every update.
        if (lastState != state) {
            lastStateChange = System.currentTimeMillis();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        if (!PROGRAMATIC_STALL_SAFETY) {
            return;
        }
        long elapsedStateMillis = System.currentTimeMillis() - lastStateChange;
        if (elapsedStateMillis > STALL_THRESHOLD) {
            double ticksPerSecond = uppiesMotor.getVelocity(AngleUnit.DEGREES);
            if (ticksPerSecond < STALL_TICKS_PER_SECOND_THRESHOLD) {
                // STALLING!!!! STOP MOVING
                setUppiesState(UppiesState.IDLE);
            }
        }
    }

    public enum UppiesState {
        UPPIES, // up
        UN_UPPIES, // down
        IDLE
    }

}
