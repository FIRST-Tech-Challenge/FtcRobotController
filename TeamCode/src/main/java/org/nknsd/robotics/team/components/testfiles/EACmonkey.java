package org.nknsd.robotics.team.components.testfiles;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.team.components.ExtensionHandler;
import org.nknsd.robotics.team.components.IntakeSpinnerHandler;
import org.nknsd.robotics.team.components.RotationHandler;

import java.util.concurrent.TimeUnit;

public class EACmonkey implements NKNComponent {
    private ExtensionHandler extensionHandler;
    private IntakeSpinnerHandler intakeSpinnerHandler;
    private RotationHandler rotationHandler;
    private Tests currentTest = Tests.DO_NOTHING;

    private int testNumber;

    private enum Tests {
        DO_NOTHING(1000),
        SERVO_FORWARD(3000),
        ROTATION_0_5(3000),
        ROTATION_1(3000),
        EXTENSION_EXTENDED(3000),
        SERVO_BACKWARD(3000),
        EXTENSION_RESTING(3000),
        ROTATION_0(3000);
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
        return "EACmonkey";
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
        if (testNumber == 1) {
            telemetry.addData("Test One", currentTest.name());
        }
    }

    private void runTest(Tests tests) {
        switch (tests) {
            case DO_NOTHING:
                intakeSpinnerHandler.setServoPower(IntakeSpinnerHandler.HandStates.TRUE_STOP);
                break;
            case SERVO_FORWARD:
                intakeSpinnerHandler.setServoPower(IntakeSpinnerHandler.HandStates.GRIP);
                break;
            case SERVO_BACKWARD:
                intakeSpinnerHandler.setServoPower(IntakeSpinnerHandler.HandStates.RELEASE);
                break;
            case EXTENSION_EXTENDED:
                extensionHandler.gotoPosition(ExtensionHandler.ExtensionPositions.HIGH_BASKET);
                break;
            case EXTENSION_RESTING:
                extensionHandler.gotoPosition(ExtensionHandler.ExtensionPositions.RESTING);
                break; 
            case ROTATION_0:
                rotationHandler.setTargetRotationPosition(RotationHandler.RotationPositions.PICKUP);
                break;
            case ROTATION_0_5:
                rotationHandler.setTargetRotationPosition(RotationHandler.RotationPositions.PREPICKUP);
                break;
            case ROTATION_1:
                rotationHandler.setTargetRotationPosition(RotationHandler.RotationPositions.HIGH);
                break;
        }
    }

    public void link(ExtensionHandler extensionHandler, IntakeSpinnerHandler intakeSpinnerHandler, RotationHandler rotationHandler) {
        this.extensionHandler = extensionHandler;
        this.intakeSpinnerHandler = intakeSpinnerHandler;
        this.rotationHandler = rotationHandler;
    }
}
