/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1617;

// import lombok.NonNull;
import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.drive.DriveUntilLineState;
import com.hfrobots.tnt.corelib.drive.GyroTurnState;
import com.hfrobots.tnt.corelib.drive.ProportionalDriveInchesStateExternalControl;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StateMachine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

@Autonomous(name="VV Auto Real")
@Disabled
@SuppressWarnings("unused")
public class VelocityVortexBeacons extends VelocityVortexHardware {

    private static final double POWER_LEVEL = 0.4;

    private static final String LOG_TAG = "TNT Auto";
    private StateMachine stateMachine = null;

    // The routes our robot knows how to do
    private enum Routes {
        CLAIM_CLOSEST_BEACON("Claim beacons"),
        PARK_ON_RAMP_1("Park on ramp 1"),
        PARK_ON_RAMP_2("Park on ramp 2"),
        PARK_ON_RAMP_3("Park on ramp 3"),
        PARK_ON_VORTEX_1("Park on center vortex 1"),
        PARK_ON_VORTEX_2("Park on center vortex 2"),
        PARK_ON_VORTEX_3("Park on center vortex 3");

        final String description;

        Routes(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    private int selectedRoutesIndex = 0;

    private Routes[] possibleRoutes = Routes.values();

    private enum Alliance { RED, BLUE };

    // Which alliance are we? (the robot is programmed from the point-of-view of the red alliance
    // but we can also have it run the blue one if selected

    private Alliance currentAlliance = Alliance.RED;

    private int initialDelaySeconds = 0;

    private boolean debugging = false;

    @Override
    public void init() {
        super.init();
        gyro.calibrate();
        beaconColorSensor.enableLed(false);
        setDefaults();
    }

    @Override
    public void start() {
        super.start();
        gyro.resetZAxisIntegrator();
        beaconColorSensor.enableLed(false);
        logBatteryState("Auto.start()");
    }

    @Override
    public void stop() {
        super.stop();
        logBatteryState("Auto.stop()");
    }

    private void setDefaults() {
        currentAlliance = Alliance.RED;
        selectedRoutesIndex = 0;
        initialDelaySeconds = 0;
    }

    private boolean configLocked = false;

    // Called repeatedly after init button has been pressed and init() has completed (we think)
    @Override
    public void init_loop() {
        if (driversGamepad == null) { // safety, need to double check whether we actually need this
            // not ready yet init() hasn't been called
            return;
        }

        if (!configLocked) {
            doAutoConfig();

            if (lockButton.getRise()) {
                configLocked = true;
            }
        } else {
            if (unlockButton.getRise()) {
                configLocked = false;
            }
        }

        if (configLocked) {
            telemetry.addData("00", "LOCKED: Press Rt stick unlock");
        } else {
            telemetry.addData("00", "UNLOCKED: Press Lt stick lock");
        }

        String mode = debugging  ? "D" : "R";

        int blueColorReading = beaconColorSensor.blue();
        int redColorReading = beaconColorSensor.red();
        int greenColorReading = beaconColorSensor.green();

        if (redColorReading == 255 && greenColorReading == 255 && blueColorReading == 255) {
            telemetry.addData("00", "** COLOR SENSOR INOP - POWER CYCLE ROBOT **", mode);
        }

        telemetry.addData("01", "[%s] Alliance: %s", mode, currentAlliance);
        telemetry.addData("02", "[%s] Route: %s", mode, possibleRoutes[selectedRoutesIndex].getDescription());
        telemetry.addData("03", "[%s] Delay %d sec", mode, initialDelaySeconds);
        telemetry.addData("04", "Color sensor: %d %d %d", redColorReading, greenColorReading, blueColorReading);
        telemetry.addData("05", "[%s] Gyro calibrating: %s", mode, Boolean.toString(gyro.isCalibrating()));

        updateTelemetry(telemetry);
    }

    private void doAutoConfig() {
        // Use driver dpad up/down to select which route to run
        if (driverDpadDown.getRise()) {
            selectedRoutesIndex--;
            if (selectedRoutesIndex < 0) {
                selectedRoutesIndex = possibleRoutes.length - 1; // why?
            }
        } else if (driverDpadUp.getRise()) {
            selectedRoutesIndex++;

            if (selectedRoutesIndex > possibleRoutes.length - 1) { // why -1?
                selectedRoutesIndex = 0;
            }
        }

        // use left/right bumper to decrease/increase delay

        if (driverLeftBumper.getRise()) {
            initialDelaySeconds -= 1;

            if (initialDelaySeconds < 0) {
                initialDelaySeconds = 0;
            }
        } else if (driverRightBumper.getRise()) {
            initialDelaySeconds += 1;

            if (initialDelaySeconds > 10) {
                initialDelaySeconds = 10;
            }
        }

        // Alliance selection
        if (driverBRedButton.getRise()) {
            currentAlliance = Alliance.RED;
        } else if (driverXBlueButton.getRise()) {
            currentAlliance = Alliance.BLUE;
        }

        // Force gyro recal
        if (driverAGreenButton.getRise()) {
            gyro.calibrate();
        }

        if (driverYYellowButton.getRise()) {
            debugging = !debugging;
        }
    }

    @Override
    public void loop() {
        try {
            if (stateMachine == null) {
                setupSelectedStateMachine();
            }

            stateMachine.doOneStateLoop();

            telemetry.update(); // send all telemetry to the drivers' station
        } catch (Throwable t) {
            // Better logging than the FTC SDK provides :(
            Log.e("VV", "Exception during state machine", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException)t;
            }

            RuntimeException rte = new RuntimeException();
            rte.initCause(t);

            throw rte;
        }
    }

    protected void setupSelectedStateMachine() {
        /* We have not configured the state machine yet, do so from the options
         selected during init_loop() */

        Routes selectedRoute = possibleRoutes[selectedRoutesIndex];

        switch (selectedRoute) {
            case CLAIM_CLOSEST_BEACON:
                stateMachine = claimBeacons();
                break;
            case PARK_ON_RAMP_1:
                stateMachine = parkOnRamp1();
                break;
            case PARK_ON_RAMP_2:
                stateMachine = parkOnRamp2();
                break;
            case PARK_ON_RAMP_3:
                stateMachine = parkOnRamp3();
                break;
            case PARK_ON_VORTEX_1:
                stateMachine = parkOnVortex1();
                break;
            case PARK_ON_VORTEX_2:
                stateMachine = parkOnVortex2();
                break;
            case PARK_ON_VORTEX_3:
                stateMachine = parkOnVortex3();
                break;
            default:
                throw new IllegalArgumentException("Invalid route selected");
        }

        if (initialDelaySeconds != 0) {
            stateMachine.addStartDelay(initialDelaySeconds);
        }

        if (debugging) {
            stateMachine.startDebugging();
        } else {
            stateMachine.stopDebugging();
        }
    }

    /**
     * Turns are relative to being in the red alliance. Because this game is exactly
     * mirror image, to get our routes working for the blue alliance we simply need to
     * reverse the direction of the turn
     */
    private Turn adjustTurnForAlliance(Turn origTurn) {
        if (currentAlliance == Alliance.RED) {
            return origTurn;
        }

        return origTurn.invert();
    }

    private StateMachine parkOnRamp3() {
        StateMachine stateMachine = commonStateMachineSetup();

        // This is only for testing now, remove when we make this real for the match
        DelayState dummyDelayState = new DelayState("Dummy", telemetry, 1, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(dummyDelayState);

        //(1)forward 5"
        ProportionalDriveInchesStateExternalControl step1DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 1 - drive forward", drive, telemetry, 5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step1DriveState);

        //(2)turn 45 CCW
        State step2TurnState = new GyroTurnState("Step 2 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 33)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step2TurnState);

        //(3)forward 32.5"
        ProportionalDriveInchesStateExternalControl step3DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 32.5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step3DriveState);

        //(4)stop and shoot
        addParticleShooterForAuto(stateMachine, new CollectorOffState(telemetry));

        //(5)turn 68.2 CCW
        State step5TurnState = new GyroTurnState("Step 5 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 50)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step5TurnState);

        //(6) forward 49"
        ProportionalDriveInchesStateExternalControl step6DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 6 - drive forward", drive, telemetry, 59 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step6DriveState);

        // (7) Reverse collector

        // (8) Drive up ramp

        // (9) Stop collector

        // (10) Turn to park

        // (11) Done
        stateMachine.addSequential(newDoneState("Park on ramp 3 done"));

        return stateMachine;
    }

    private StateMachine parkOnRamp1() {
        StateMachine stateMachine = commonStateMachineSetup();

        // This is only for testing now, remove when we make this real for the match
        DelayState dummyDelayState = new DelayState("Dummy", telemetry, 1, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(dummyDelayState);

        // (1) Drive forward 16"
        ProportionalDriveInchesStateExternalControl step1DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 1 - drive forward", drive, telemetry, 20 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step1DriveState);

        // (2) Shoot
        addParticleShooterForAuto(stateMachine, new CollectorOffState(telemetry));

        // (3) Turn 97 degrees CCW
        State step3TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 97)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step3TurnState);

        // (4) Forward 26"
        ProportionalDriveInchesStateExternalControl step4DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 1 - drive forward", drive, telemetry, 32 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step4DriveState);

        // (5) Square up with ramp (not done yet)


        // (6) Reverse collector


        // (7) Drive up ramp

        // (8) Stop collector

        // (9) Turn to park

        // (10) Done

        stateMachine.addSequential(newDoneState("Park on ramp 1 done"));

        return stateMachine;
    }

    private StateMachine parkOnRamp2() {
        StateMachine stateMachine = commonStateMachineSetup();

        // This is only for testing now, remove when we make this real for the match
        DelayState dummyDelayState = new DelayState("Dummy", telemetry, 1, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(dummyDelayState);

        // (1) Drive forward 25.5"
        ProportionalDriveInchesStateExternalControl step1DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 1 - drive forward", drive, telemetry, 25.5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step1DriveState);


        // (2) Turn 45 degrees CCW
        State step2TurnState = new GyroTurnState("Step 2 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 33)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step2TurnState);

        // (3) Shoot
        addParticleShooterForAuto(stateMachine, new CollectorOffState(telemetry));

        // (3) Turn 64 degrees CCW
        State step3TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 44)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step3TurnState);

        // (4) Forward 51"
        ProportionalDriveInchesStateExternalControl step4DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 1 - drive forward", drive, telemetry, 58 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step4DriveState);

        // (5) Square up with ramp (not done yet)

        // (6) Reverse collector

        // (7) Drive up ramp

        // (8) Stop collector

        // (9) Turn to park

        // (10) Done

        stateMachine.addSequential(newDoneState("Park on ramp 3 done"));

        return stateMachine;
    }

    private StateMachine parkOnVortex1() {
        StateMachine stateMachine = commonStateMachineSetup();

        // This is only for testing now, remove when we make this real for the match
        DelayState dummyDelayState = new DelayState("Dummy", telemetry, 1, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(dummyDelayState);

        // (1) Drive forward 16"

        ProportionalDriveInchesStateExternalControl step1DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 1 - drive forward", drive, telemetry, 20 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step1DriveState);


        // (2) Particle Shoot
        // (2a) - Need a state for "waiting" for button press, a DelayState?

        addParticleShooterForAuto(stateMachine, new CollectorOffState(telemetry));

        // (3) Drive forward 16.5"
        ProportionalDriveInchesStateExternalControl step4DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 12.5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step4DriveState);


        // (4) Turn 90 degrees CCW (and back) to clear the cap ball

        State step3TurnState = new GyroTurnState("Step 4 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 90)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step3TurnState);


        // (5) Turn 90 degrees CW
        State step5TurnState = new GyroTurnState("Step 5 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CW, 90)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step5TurnState);

        // (6) Forward 18"
        ProportionalDriveInchesStateExternalControl step6DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 6 - drive forward", drive, telemetry, 20 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step6DriveState);

        // (8) Done
        stateMachine.addSequential(newDoneState("Park on vortex 1 done"));

        return stateMachine;
    }

    private StateMachine parkOnVortex2() {
        StateMachine stateMachine = commonStateMachineSetup();

        // This is only for testing now, remove when we make this real for the match
        DelayState dummyDelayState = new DelayState("Dummy", telemetry, 1, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(dummyDelayState);

        // (1) Forward 25.5"
        ProportionalDriveInchesStateExternalControl step1DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 25.5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step1DriveState);

        // (2) Turn 45 deg ccw
        State step2TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 33)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step2TurnState);

        // (3) Shoot
        addParticleShooterForAuto(stateMachine, new CollectorOffState(telemetry));

        // (4) Forward 14"
        ProportionalDriveInchesStateExternalControl step4DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 14 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step4DriveState);

        // (5) Turn 45 degrees ccw
        State step5TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 45)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step5TurnState);

        // (6) Turn 45 degrees cw
        State step6TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CW, 45)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step6TurnState);

        // (7) Forward 3"
        ProportionalDriveInchesStateExternalControl step7DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 10 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step7DriveState);

        // (8) Done
        stateMachine.addSequential(newDoneState("Park on vortex 2 done"));

        return stateMachine;
    }

    private StateMachine parkOnVortex3() {
        StateMachine stateMachine = commonStateMachineSetup();

        // This is only for testing now, remove when we make this real for the match
        DelayState dummyDelayState = new DelayState("Dummy", telemetry, 1, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(dummyDelayState);

        // (1) Forward 5"
        ProportionalDriveInchesStateExternalControl step1DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step1DriveState);

        // (2) Turn 45 deg ccw
        State step2TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 33)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step2TurnState);

        // (3) Forward 32.5"
        ProportionalDriveInchesStateExternalControl step3DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 32.5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step3DriveState);


        // (4) Shoot
        addParticleShooterForAuto(stateMachine, new CollectorOffState(telemetry));

        // (5) Forward 12.5"
        ProportionalDriveInchesStateExternalControl step5DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 12.5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step5DriveState);

        // (6) Turn 45 deg ccw
        State step6TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 45)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step6TurnState);

        // (7) Turn 45 deg cw
        State step7TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CW, 45)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step7TurnState);

        // (8) Forward 7"
        ProportionalDriveInchesStateExternalControl step8DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 3 - drive forward", drive, telemetry, 7 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step8DriveState);

        // (8) Done
        stateMachine.addSequential(newDoneState("Park on vortex 3 done"));

        return stateMachine;
    }

    private StateMachine claimBeacons() {
        if (currentAlliance.equals(Alliance.RED)) {
            return claimBeaconsRed();
        } else {
            return claimBeaconsBlue();
        }
    }

    private StateMachine claimBeaconsRed() {
        StateMachine stateMachine = commonStateMachineSetup();

        // This is only for testing now, remove when we make this real for the match
        DelayState dummyDelayState = new DelayState("Dummy", telemetry, 1, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(dummyDelayState);

        // (1) Drive forward 16"

        ProportionalDriveInchesStateExternalControl step1DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 1 - drive forward", drive, telemetry, 20 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step1DriveState);


        // (2) Particle Shoot
        // (2a) - Need a state for "waiting" for button press, a DelayState?
        // Collector off, it vibrates the gyro

        addParticleShooterForAuto(stateMachine, new CollectorOffState(telemetry));

        // (3) Turn 50 degrees CCW
        State step3TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 49)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step3TurnState);

        // TODO: From here on, power is - when blue alliance because robot is running backwards

        // (4) Drive forward 46"
        ProportionalDriveInchesStateExternalControl step4DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 4 - drive forward", drive, telemetry, 47.5 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step4DriveState);

        // (5) Turn 60 degrees CW
        State step5TurnState = new GyroTurnState("Step 5 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CW, 41)), // don't come all the way ||
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step5TurnState);

        // (6) Forward 18" (should be in place to press beacons)
        ProportionalDriveInchesStateExternalControl step6DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 6 - drive forward", drive, telemetry, 2 /*and w/ ods 8 inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step6DriveState);


        DriveUntilLineState step7UntilLineState = new DriveUntilLineState("Drive until beacon line", drive,
                telemetry, inboardLineSensor, outboardLineSensor, gyro, POWER_LEVEL / 3, 15000);
        stateMachine.addSequential(step7UntilLineState);

        ProportionalDriveInchesStateExternalControl step7SmidgeState = new ProportionalDriveInchesStateExternalControl(
                "State 7 - smidge forward", drive, telemetry, .25,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step7SmidgeState);

        // (7) Detect beacon color and press button
        stateMachine.addSequential(new BeaconPusherState(telemetry));

        stateMachine.addSequential(new DelayState("waiting for beacon push", telemetry, 4));
        stateMachine.addSequential(new PusherRetractState(telemetry));
        stateMachine.addSequential(new DelayState("waiting for beacon retract", telemetry, 1));

        // FIXME: If we're getting penalty (wrong) beacons by brush-by, reduce this angle!
        State step7TurnState = new GyroTurnState("Step 7 turn", drive,
                gyro,
                adjustTurnForAlliance(new Turn(Rotation.CCW, 5)),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step7TurnState);

        // TOD O absolute gyro turn? Getting square up working may fix this
        // (8)drive 31 inches
        ProportionalDriveInchesStateExternalControl step8DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 6 - drive forward", drive, telemetry, 31 ,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step8DriveState);

        // (9) drive until far beacon line
        DriveUntilLineState step9UntilLineState = new DriveUntilLineState("Drive until next beacon line", drive,
                telemetry, inboardLineSensor, outboardLineSensor, gyro, POWER_LEVEL / 3, 15000);
        stateMachine.addSequential(step9UntilLineState);

        ProportionalDriveInchesStateExternalControl step9SmidgeState = new ProportionalDriveInchesStateExternalControl(
                "State 9 - smidge forward", drive, telemetry, .25,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step9SmidgeState);

        // (10) detect beacon color and press beacon
        stateMachine.addSequential(new BeaconPusherState(telemetry));

        stateMachine.addSequential(new DelayState("waiting for beacon push", telemetry, 4));
        stateMachine.addSequential(new PusherRetractState(telemetry));

        // (11) Done
        stateMachine.addSequential(newDoneState("Beacon press done"));

        return stateMachine;
    }

    private StateMachine claimBeaconsBlue() {
        StateMachine stateMachine = commonStateMachineSetup();

        // This is only for testing now, remove when we make this real for the match
        DelayState dummyDelayState = new DelayState("Dummy", telemetry, 1, TimeUnit.MILLISECONDS);
        stateMachine.addSequential(dummyDelayState);

        // (1) Drive forward 16"

        ProportionalDriveInchesStateExternalControl step1DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 1 - drive forward", drive, telemetry, 20 /* inches */,
                POWER_LEVEL /* power level*/, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step1DriveState);


        // (2) Particle Shoot
        // (2a) - Need a state for "waiting" for button press, a DelayState?
        // Collector off, it vibrates the gyro

        addParticleShooterForAuto(stateMachine, new CollectorOffState(telemetry));

        // (3) Turn 50 degrees CCW
        // 50 really gets us 60

        State step3TurnState = new GyroTurnState("Step 3 turn", drive,
                gyro,
                new Turn(Rotation.CCW, 110),
                telemetry,
                POWER_LEVEL / 1.5,
                20000L);
        stateMachine.addSequential(step3TurnState);

        // (4) Drive forward 46"
        ProportionalDriveInchesStateExternalControl step4DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 4 - drive forward", drive, telemetry, 42.5 /* inches */,
                POWER_LEVEL /* power level*/, DcMotorSimple.Direction.REVERSE, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step4DriveState);

        // (5) Turn 60 degrees CW
        State step5TurnState = new GyroTurnState("Step 5 turn", drive,
                gyro,
                new Turn(Rotation.CCW, 45),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step5TurnState);

        // (6) Forward 18" (should be in place to press beacons)
        ProportionalDriveInchesStateExternalControl step6DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 6 - drive forward", drive, telemetry, 3 /*and w/ ods 8 inches */,
                POWER_LEVEL /* power level*/, DcMotorSimple.Direction.REVERSE, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step6DriveState);

        DriveUntilLineState step7UntilLineState = new DriveUntilLineState("Drive until beacon line", drive,
                telemetry, inboardLineSensor, outboardLineSensor, gyro, -POWER_LEVEL / 3, 15000);
        stateMachine.addSequential(step7UntilLineState);

        ProportionalDriveInchesStateExternalControl step7SmidgeState = new ProportionalDriveInchesStateExternalControl(
                "State 7 - smidge forward", drive, telemetry, .25,
                POWER_LEVEL /* power level*/, DcMotorSimple.Direction.FORWARD, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step7SmidgeState);

        // (7) Detect beacon color and press button
        stateMachine.addSequential(new BeaconPusherState(telemetry));

        stateMachine.addSequential(new DelayState("waiting for beacon push", telemetry, 4));
        stateMachine.addSequential(new PusherRetractState(telemetry));
        stateMachine.addSequential(new DelayState("waiting for beacon retract", telemetry, 1));

        State step7TurnState = new GyroTurnState("Step 7 turn", drive,
                gyro,
                new Turn(Rotation.CW, 5),
                telemetry,
                POWER_LEVEL,
                20000L);
        stateMachine.addSequential(step7TurnState);

        // TOD O absolute gyro turn? Getting square up working may fix this
        // (8)drive 31 inches
        ProportionalDriveInchesStateExternalControl step8DriveState = new ProportionalDriveInchesStateExternalControl(
                "State 6 - drive forward", drive, telemetry, 31 ,
                POWER_LEVEL /* power level*/, DcMotorSimple.Direction.REVERSE, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step8DriveState);

        // (9) drive until far beacon line
        DriveUntilLineState step9UntilLineState = new DriveUntilLineState("Drive until next beacon line", drive,
                telemetry, inboardLineSensor, outboardLineSensor, gyro, -POWER_LEVEL / 3, 15000);
        stateMachine.addSequential(step9UntilLineState);

        // We're running in reverse! So, the sensors hit late, forward is actually "reverse" in this case
        ProportionalDriveInchesStateExternalControl step9SmidgeState = new ProportionalDriveInchesStateExternalControl(
                "State 9 - smidge forward", drive, telemetry, .25,
                POWER_LEVEL /* power level*/, DcMotorSimple.Direction.FORWARD, 15000 /* milliseconds to timeout */);
        stateMachine.addSequential(step9SmidgeState);

        // (10) detect beacon color and press beacon
        stateMachine.addSequential(new BeaconPusherState(telemetry));

        stateMachine.addSequential(new DelayState("waiting for beacon push", telemetry, 4));
        stateMachine.addSequential(new PusherRetractState(telemetry));

        // (11) Done
        stateMachine.addSequential(newDoneState("Beacon press done"));

        return stateMachine;
    }

    @NonNull
    private StateMachine commonStateMachineSetup() {
        StateMachine stateMachine = new StateMachine(telemetry);

        // Setup debugger controls
        stateMachine.setDoOverButton(driverBRedButton);
        stateMachine.setGoBackButton(driverYYellowButton);
        stateMachine.setGoButton(driverAGreenButton);
        stateMachine.setConfigureGamepad(operatorsGamepad);

        return stateMachine;
    }


    private void addParticleShooterForAuto(StateMachine stateMachine, State collectorEndState) {
        State step2aSettleState = new DelayState("Wait to shoot", telemetry,
                250, TimeUnit.MILLISECONDS);

        // (2b) - Need a state for waiting to "release" the button, another DelayState?

        State step2bWaitForParticlesState = new DelayState("Wait to shoot", telemetry,
                3, TimeUnit.SECONDS);

        // PEW PEW!
        addShooterStateMachine(stateMachine, step2aSettleState, step2bWaitForParticlesState, collectorEndState, false);
    }

    class PusherRetractState extends State {
        public PusherRetractState(Telemetry telemetry) {
            super("Retract beacons", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            beaconPusherNoColorSensor.setPosition(BEACON_PUSHER_IN_POSITION);
            beaconPusherUnderColorSensor.setPosition(BEACON_PUSHER_IN_POSITION);
            return nextState;
        }

        protected PusherRetractState(String name, Telemetry telemetry) {
            super(name, telemetry);
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }

        @Override
        public void resetToStart() {
            beaconPusherNoColorSensor.setPosition(BEACON_PUSHER_OUT_POSITION);
            beaconPusherUnderColorSensor.setPosition(BEACON_PUSHER_OUT_POSITION);
        }
    }

    class BeaconPusherState extends State {
        public BeaconPusherState(Telemetry telemetry) {
            super("BeaconPusher", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            int blueColorReading = beaconColorSensor.blue();
            int redColorReading = beaconColorSensor.red();
            int greenColorReading = beaconColorSensor.green();

            // Better not get this wrong, it's 30 points for the other team if you do

            // TODO: What can/should we do to determine if color sensor isn't sensing correctly?

            // "in" on the linear servo is .setPosition(0)
            // "push" on the linear servo is .setPosition(.60somethingorother).
            // if (seeRed) { // push something } else if (seeBlue) { // push something} else { // error }

            if (isSensingRedColor(redColorReading, greenColorReading, blueColorReading)) {
                if (currentAlliance == Alliance.RED) {
                    Log.d("VV", "Saw red beacon, red alliance, pushing beacon under color sensor");
                    beaconPusherUnderColorSensor.setPosition(BEACON_PUSHER_OUT_POSITION);
                    beaconPusherNoColorSensor.setPosition(0);
                } else {
                    // blue alliance
                    Log.d("VV", "Saw red beacon, blue alliance, pushing beacon *not* under color sensor");
                    beaconPusherUnderColorSensor.setPosition(0);
                    beaconPusherNoColorSensor.setPosition(BEACON_PUSHER_OUT_POSITION);
                }
            } else if (isSensingBlueColor(redColorReading, greenColorReading, blueColorReading)) {
                if (currentAlliance == Alliance.BLUE) {
                    Log.d("VV", "Saw blue beacon, blue alliance, pushing beacon under color sensor");
                    beaconPusherUnderColorSensor.setPosition(BEACON_PUSHER_OUT_POSITION);
                    beaconPusherNoColorSensor.setPosition(0);
                } else {
                    //red alliance
                    Log.d("VV", "Saw blue beacon, red alliance, pushing beacon *not* under color sensor");
                    beaconPusherUnderColorSensor.setPosition(0);
                    beaconPusherNoColorSensor.setPosition(BEACON_PUSHER_OUT_POSITION);
                }
            } else {
                Log.d("VV", "Did not see red or blue, not pushing beacon, observed values "
                        + redColorReading + ", " + greenColorReading + ", " + blueColorReading);
            }

            return nextState;
        }

        protected boolean isSensingBlueColor(int redColorReading,
                                             int greenColorReading,
                                             int blueColorReading) {
            Log.d("VV", "isSensingRed (RGB) " + redColorReading + ", " + greenColorReading +  ", " + blueColorReading);

            return blueColorReading > redColorReading && blueColorReading > greenColorReading;
        }

        protected boolean isSensingRedColor(int redColorReading,
                                            int greenColorReading,
                                            int blueColorReading) {
            Log.d("VV", "isSensingBlue (RGB) " + redColorReading + ", " + greenColorReading + ", " + blueColorReading);

            return redColorReading > blueColorReading && redColorReading > greenColorReading;
        }

        @Override
        public void resetToStart() {
            beaconPusherNoColorSensor.setPosition(BEACON_PUSHER_IN_POSITION);
            beaconPusherUnderColorSensor.setPosition(BEACON_PUSHER_IN_POSITION);
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {

        }
    }
}
