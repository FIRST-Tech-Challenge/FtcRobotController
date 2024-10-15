package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

import java.util.concurrent.TimeUnit;

public class ChaosMonkey implements NKNComponent {
    // The states enum stores the possible tests chaos monkey can run
    // Each one has a duration, and later we define which state runs what
    private enum States {
        DO_NOTHING(1000) {
            @Override
            public void runTest() {

            }
        },
        FORWARD(1000) {
            @Override
            public void runTest() {

            }
        },
        BACKWARDS(1000) {
            @Override
            public void runTest() {

            }
        },
        LEFT(1000) {
            @Override
            public void runTest() {

            }
        },
        RIGHT(1000) {
            @Override
            public void runTest() {

            }
        },
        CLOCKWISE(1000) {
            @Override
            public void runTest() {

            }
        },
        COUNTERCLOCKWISE(1000) {
            @Override
            public void runTest() {

            }
        },
        MOTORBR(1000) {
            @Override
            public void runTest() {

            }
        },
        MOTORBL(1000) {
            @Override
            public void runTest() {

            }
        },
        MOTORFR(1000) {
            @Override
            public void runTest() {

            }
        },
        MOTORFL(1000) {
            @Override
            public void runTest() {

            }
        };

        public final long durationMilli;

        States(long durationMilli) {
            this.durationMilli = durationMilli;
        }

        public abstract void runTest();
    }

    private States state = States.DO_NOTHING;
    private long stateStartTime = 0; // Stores the clock time at which the test started to run, so that we can stop it after the delay

    private final WheelHandler wheelHandler;

    public ChaosMonkey(WheelHandler wheelHandler, String[] doTests) {
        this.wheelHandler = wheelHandler;

    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {return true;}

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public String getName() {return "ChaosMonkey";}


    private void runState() {
//        if (!shouldDoState(state)) {return;}

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

            case MOTORBR:
                wheelHandler.vectorToMotion(0, 0, 0);
                wheelHandler.runMotorBR(0.5);
                break;

            case MOTORBL:
                wheelHandler.vectorToMotion(0, 0, 0);
                wheelHandler.runMotorBL(0.5);
                break;

            case MOTORFR:
                wheelHandler.vectorToMotion(0, 0, 0);
                wheelHandler.runMotorFR(0.5);
                break;

            case MOTORFL:
                wheelHandler.vectorToMotion(0, 0, 0);
                wheelHandler.runMotorFL(0.5);
                break;
        }
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        // Check if the current time is higher than the target time to stop
        if (runtime.time(TimeUnit.MILLISECONDS) - stateStartTime > state.durationMilli) {
            // Get which test we're on
            int stateIndex = state.ordinal();

            // If outside the array's length, we need to loop back to the beginning
            if (stateIndex >= States.values().length - 1) { // We have to use length - 1 since we add 1 to index later
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
