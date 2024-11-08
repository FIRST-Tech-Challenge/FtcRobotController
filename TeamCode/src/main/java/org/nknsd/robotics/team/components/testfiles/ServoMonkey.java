package org.nknsd.robotics.team.components.testfiles;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;

import java.util.concurrent.TimeUnit;

public class ServoMonkey implements NKNComponent {
    private IntakeSpinnerHandler intakeSpinnerHandler;
    private Tests currentTest = Tests.DO_NOTHING;

    private enum Tests {
        DO_NOTHING(3000),
        TARGET_FORWARD(3000),
        TARGET_BACKWARD(3000);


        public final long durationMilli;

        Tests(long durationMilli) {
            this.durationMilli = durationMilli;
        }
    }

    private long testStartTime = 0;

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
    public String getName() {
        return "ServoMonkey";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        // Check if the current time is higher than the target time to stop
        if (runtime.time(TimeUnit.MILLISECONDS) - testStartTime > currentTest.durationMilli) {
            // Get which test we're on
            int stateIndex = currentTest.ordinal();

            // If outside the array's length, we need to loop back to the beginning
            if (stateIndex >= Tests.values().length - 1) { // We have to use length - 1 since we add 1 to index later
                //Set stateIndex to -1 so we target the first index when we add 1
                stateIndex = -1;
            }

            currentTest = Tests.values()[stateIndex + 1];

            testStartTime = runtime.time(TimeUnit.MILLISECONDS);
        }

        runTest(currentTest);
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Test", currentTest.name());
    }

    private void runTest(Tests tests) {
        switch (tests) {
            case DO_NOTHING:
                intakeSpinnerHandler.setServoPower(IntakeSpinnerHandler.HandStates.TRUE_STOP);
                break;

            case TARGET_FORWARD:
                intakeSpinnerHandler.setServoPower(IntakeSpinnerHandler.HandStates.GRIP);
                break;

            case TARGET_BACKWARD:
                intakeSpinnerHandler.setServoPower(IntakeSpinnerHandler.HandStates.RELEASE);
                break;
        }
    }

    public void link(IntakeSpinnerHandler intakeSpinnerHandler) {
        this.intakeSpinnerHandler = intakeSpinnerHandler;
    }
}
