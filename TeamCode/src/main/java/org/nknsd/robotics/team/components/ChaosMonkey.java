package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

import java.util.concurrent.TimeUnit;

public class ChaosMonkey implements NKNComponent {
    private enum States {
        DO_NOTHING(1000),
        FORWARD(1000),
        BACKWARDS(1000),
        LEFT(1000),
        RIGHT(1000),
        CLOCKWISE(1000),
        COUNTERCLOCKWISE(1000);

        public final long durationMilli;

        States(long durationMilli) {
            this.durationMilli = durationMilli;
        }

    }

    private States state = States.DO_NOTHING;
    private long stateStartTime = 0;

    private final WheelHandler wheelHandler;

    public ChaosMonkey(WheelHandler wheelHandler) {
        this.wheelHandler = wheelHandler;
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {return "ChaosMonkey";}

    private void runState() {
        switch (state) {
            case DO_NOTHING:
                wheelHandler.vectorToMotion(0, 0, 0);
                break;

            case FORWARD:
                wheelHandler.vectorToMotion(0.5f, 0, 0);
                break;

            case BACKWARDS:
                wheelHandler.vectorToMotion(-0.5f, 0, 0);
                break;

            case LEFT:
                wheelHandler.vectorToMotion(0, -0.5f, 0);
                break;

            case RIGHT:
                wheelHandler.vectorToMotion(0, 0.5f, 0);
                break;

            case CLOCKWISE:
                wheelHandler.vectorToMotion(0, 0, 0.5f);
                break;

            case COUNTERCLOCKWISE:
                wheelHandler.vectorToMotion(0, 0, -0.5f);
                break;
        }
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        // If time duration has passed, increment state
        // if state is > last state, go back to state 0
        // run function for each state

        if (runtime.time(TimeUnit.MILLISECONDS) - stateStartTime > state.durationMilli) {
            int stateIndex = state.ordinal();

            if (stateIndex >= States.values().length - 1) {
                //Set stateIndex to -1 so we target the first index when we add 1
                stateIndex = -1;
            }

            state = States.values()[stateIndex + 1];
            stateStartTime = runtime.time(TimeUnit.MILLISECONDS);
        }

        runState();

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("State", state.name());
    }
}
