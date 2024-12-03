package org.nknsd.teamcode.components.testfiles;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;
import org.nknsd.teamcode.components.handlers.ExtensionHandler;

import java.util.concurrent.TimeUnit;

public class ExtensionMonkey implements NKNComponent {
    private ExtensionHandler extensionHandler;
    private Tests currentTest = Tests.DO_NOTHING;

    private enum Tests {
        DO_NOTHING(1000),
        RESTING_POSITION(3000),
        EXTENDED_POSITION(3000);


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
        return "ExtensionMonkey";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        // Check if the current time is higher than the target time to stop
        if (runtime.time(TimeUnit.MILLISECONDS) - testStartTime > currentTest.durationMilli) {
            // Get which test we're on
            int testIndex = currentTest.ordinal() + 1;

            // If outside the array's length, we need to loop back to the beginning
            if (testIndex >= Tests.values().length) {
                //Reset to beginning of tests
                testIndex = 0;
            }

            currentTest = Tests.values()[testIndex];

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
                break;

            case RESTING_POSITION:
                extensionHandler.gotoPosition(ExtensionHandler.ExtensionPositions.RESTING);
                break;

            case EXTENDED_POSITION:
                extensionHandler.gotoPosition(ExtensionHandler.ExtensionPositions.HIGH_BASKET);
                break;
        }
    }

    public void link(ExtensionHandler extensionHandler) {
        this.extensionHandler = extensionHandler;
    }
}
