package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.function.BooleanSupplier;

@Config
public class UppiesSubsystem extends SubsystemBase {

    DcMotorEx uppiesMotor;
    private UppiesState state;
    long lastStateChange = 0;
    static float speedMultiplier = .35f;

    private final static FTCDashboardPackets dbp = new FTCDashboardPackets("UppiesSubsytem");
    private final static FTCDashboardPackets dbp2 = new FTCDashboardPackets("DebugPositionUppies");

    public static boolean PROGRAMATIC_STALL_SAFETY = false;
    public static boolean PROGRAMATIC_IGNORE_LIMITS = false;
    // After the motor has enough time to start moving, start checking if the motor is stalling.
    private static final long STALL_THRESHOLD = 300;
    // Once the motor is moving less than the specified ticks per second, it is stalling.
    public static final double STALL_TICKS_PER_SECOND_THRESHOLD = .5;

    public static int MAX_POSITION = -33;
    public static int MIN_POSITION = -5000;

    public UppiesSubsystem(DcMotorEx uppiesMotor) {
        this.uppiesMotor = uppiesMotor;
        this.state = UppiesState.IDLE;
        uppiesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uppiesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uppiesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setUppiesState(final UppiesState state) {
        dbp.info("Updating state to: " + state.toString());
        UppiesState lastState = this.state;
        this.state = state;
        // TODO: Verify that this is the correct direction
        uppiesMotor.setPower(state == UppiesState.UN_UPPIES ? speedMultiplier : -speedMultiplier);
        if (state == UppiesState.IDLE) {
            uppiesMotor.setPower(0);
        }
        // If the state changed, reset the timer. Allows for this method to be called periodically/every update.
        if (lastState != this.state) {
            lastStateChange = System.currentTimeMillis();
        }
        dbp.send(true);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (state == UppiesState.IDLE) {
            return;
        }

        int currentPosition = uppiesMotor.getCurrentPosition();
        dbp2.info("Uppies Position: " + currentPosition);
        dbp2.send(true);

        if (!PROGRAMATIC_IGNORE_LIMITS && state == UppiesState.UPPIES && currentPosition <= MIN_POSITION) {
            setUppiesState(UppiesState.IDLE);
            dbp.info("EXCEEDED MAX LIMIT. HALTING.");
            dbp.send(true);
        }

        if (!PROGRAMATIC_IGNORE_LIMITS && state == UppiesState.UN_UPPIES && currentPosition >= MAX_POSITION) {
            setUppiesState(UppiesState.IDLE);
            dbp.info("EXCEEDED MIN LIMIT. HALTING.");
            dbp.send(true);
        }

        if (!PROGRAMATIC_STALL_SAFETY) {
            return;
        }

        long elapsedStateMillis = System.currentTimeMillis() - lastStateChange;
        if (elapsedStateMillis > STALL_THRESHOLD) {
            double ticksPerSecond = uppiesMotor.getVelocity(AngleUnit.DEGREES);
            dbp.info("Ticks = "+ticksPerSecond);
            dbp.send(false);
            if (ticksPerSecond < STALL_TICKS_PER_SECOND_THRESHOLD) {
                // STALLING!!!! STOP MOVING
                setUppiesState(UppiesState.IDLE);
                dbp.warn("UPPIES STALLING!");
                dbp.send(true);
            }
        }
    }

    public enum UppiesState {
        UPPIES, // up
        UN_UPPIES, // down
        IDLE
    }

    /**
     * @author Carter
     */
    public void IWantUp() {
        setUppiesState(UppiesState.UPPIES);
    }

    public void IWantDown() {
        setUppiesState(UppiesState.UN_UPPIES);
    }

    public boolean isIdle() {
        return this.state.equals(UppiesState.IDLE);
    }
}
