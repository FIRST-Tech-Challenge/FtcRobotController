/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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
 */

package com.hfrobots.tnt.season2021;

import android.widget.ToggleButton;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.ToggledButton;
import com.ftc9929.corelib.state.State;
import com.ftc9929.testing.fakes.FakeTelemetry;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.control.FakeRangeInput;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.drive.FakeServo;
import com.google.common.testing.FakeTicker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ScoringMechanismTest {
    private FakeDcMotorEx intakeMotor;

    private FakeDcMotorEx frontLauncherMotor;

    private FakeDcMotorEx rearLauncherMotor;

    private FakeServo ringFeedServo;

    private FakeServo hopperPullDownServo;

    private FakeTicker fakeTicker = new FakeTicker();
    private FakeTelemetry fakeTelemetry = new FakeTelemetry();
    private FakeRangeInput intakeVelocity = new FakeRangeInput();
    private FakeOnOffButton launchTrigger = new FakeOnOffButton();
    private FakeOnOffButton toSpeedOnOff = new FakeOnOffButton();
    private FakeOnOffButton stopLaunchingButton = new FakeOnOffButton();
    private FakeOnOffButton unsafeButton = new FakeOnOffButton();

    // "Derived"
    private DebouncedButton stopLaunchingDebounced = stopLaunchingButton.debounced();
    private ToggledButton toSpeedToggleButton = new ToggledButton(toSpeedOnOff);

    private ScoringMechanism scoringMechanism;

    @Before
    public void setup() {
        scoringMechanism = ScoringMechanism.builder()
                .hardwareMap(UltimateGoalTestConstants.HARDWARE_MAP)
                .intakeVelocity(intakeVelocity)
                .launchTrigger(launchTrigger)
                .telemetry(fakeTelemetry).ticker(fakeTicker).build();

        scoringMechanism.setUpToSpeedToggle(toSpeedToggleButton);
        scoringMechanism.setUnsafe(unsafeButton);
        scoringMechanism.setStopLauncher(stopLaunchingDebounced);

        intakeMotor = (FakeDcMotorEx) UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "intakeMotor");
        frontLauncherMotor = (FakeDcMotorEx) UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "frontLauncherMotor");
        rearLauncherMotor = (FakeDcMotorEx) UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "rearLauncherMotor");

        ringFeedServo = (FakeServo) UltimateGoalTestConstants.HARDWARE_MAP.get(Servo.class, "ringFeedServo");
        hopperPullDownServo = (FakeServo) UltimateGoalTestConstants.HARDWARE_MAP.get(Servo.class, "hopperPullDownServo");
    }

    @Test
    public void testIntakeToFiring() {
        State shouldBeIdleState = scoringMechanism.getCurrentState();
        assertEquals(ScoringMechanism.IdleState.class, shouldBeIdleState.getClass());

        backwardsOnInput(intakeVelocity, .5F);

        scoringMechanism.periodicTask();

        State shouldBeIntaking = scoringMechanism.getCurrentState();
        assertEquals(ScoringMechanism.IntakeMoving.class, shouldBeIntaking.getClass());
    }

    @Test
    public void intakeExpectedStatesWhileMoving() {
        State shouldBeIdleState = scoringMechanism.getCurrentState();
        assertEquals(ScoringMechanism.IdleState.class, shouldBeIdleState.getClass());

        backwardsOnInput(intakeVelocity, .5F);

        scoringMechanism.periodicTask();

        assertEquals(ScoringMechanism.IntakeMoving.class, scoringMechanism.getCurrentState().getClass());

        scoringMechanism.periodicTask();

        assertEquals(ScoringMechanism.IntakeMoving.class, scoringMechanism.getCurrentState().getClass());
    }

    @Test
    @Ignore // Intake encoders not installed, yet
    public void intakeStallDetector() {
        State shouldBeIdleState = scoringMechanism.getCurrentState();
        assertEquals(ScoringMechanism.IdleState.class, shouldBeIdleState.getClass());

        // Run this sequence of events more than once, to test whether the timer used for
        // timing out actually resets itself
        for (int i = 0; i < 2; i++) {
            backwardsOnInput(intakeVelocity, .5F);

            scoringMechanism.periodicTask();

            assertEquals(ScoringMechanism.IntakeMoving.class, scoringMechanism.getCurrentState().getClass());

            // The next 3 times through the state machine, pretend the motor does not move by
            // setting the encoder position to the same value. It takes 3 times through the stall detector
            // to detect a stall...

            intakeMotor.setCurrentPosistion(42);
            scoringMechanism.periodicTask();
            fakeTicker.advance(1100, TimeUnit.MILLISECONDS);
            scoringMechanism.periodicTask();
            fakeTicker.advance(1100, TimeUnit.MILLISECONDS);
            scoringMechanism.periodicTask();

            // At this point, we should be in the IntakeStalled state, and the intake motor should be
            // stopped (zero power)
            assertEquals(ScoringMechanism.IntakeStalled.class, scoringMechanism.getCurrentState().getClass());
            scoringMechanism.periodicTask(); // actually do the logic in the state

            // Make sure we're still in the stalled state
            assertEquals(ScoringMechanism.IntakeStalled.class, scoringMechanism.getCurrentState().getClass());

            // Intake motor should be zero power
            assertEquals(0, intakeMotor.getPower(), 0.05);

            // Move past the timeout - to see if we transition back to idle
            fakeTicker.advance(1100, TimeUnit.MILLISECONDS);
            scoringMechanism.periodicTask();
            assertEquals(ScoringMechanism.IdleState.class, scoringMechanism.getCurrentState().getClass());
        }
    }

    @Test
    public void launcherHappyPath() {
        State shouldBeIdleState = scoringMechanism.getCurrentState();
        assertEquals(ScoringMechanism.IdleState.class, shouldBeIdleState.getClass());
        scoringMechanism.periodicTask();


        assertTrue(frontLauncherMotor.getVelocity() > 0 && frontLauncherMotor.getVelocity() < 1000);
        assertTrue(rearLauncherMotor.getVelocity() > 0 && rearLauncherMotor.getVelocity() < 1000);

        assertEquals(hopperPullDownServo.getPosition(), Launcher.HOPPER_PULLED_DOWN_POSITION, 0.05);
        assertEquals(ringFeedServo.getPosition(), Launcher.RING_FEEDER_PARKED_POSITION, 0.05);

        // FIXME: What other things should be asserted in idle state?

        // Hit the bumper to bring the launcher up to speed
        causeButtonToRiseForNextPeriodTask(toSpeedOnOff);
        scoringMechanism.periodicTask();

        assertTrue(frontLauncherMotor.getVelocity() > 2000);
        assertTrue(rearLauncherMotor.getVelocity() > 2000);

        // Move into "Launch Mode"
        launchTrigger.setPressed(true);
        scoringMechanism.periodicTask();
        scoringMechanism.periodicTask();
        assertEquals(Launcher.HOPPER_FLOAT_POSITION, hopperPullDownServo.getPosition(), 0.05);

        fakeTicker.advance(5000, TimeUnit.MILLISECONDS);
        scoringMechanism.periodicTask();

        // FIXME: Assert what state things should be in, if launch requested, but launcher not ready

        // Get launcher up to speed (set encoder positions)
        // this takes at least 3 periodicTasks...(see launcher test)
        frontLauncherMotor.setCurrentPosistion(1000);
        rearLauncherMotor.setCurrentPosistion(1000);
        scoringMechanism.periodicTask();

        fakeTicker.advance(120, TimeUnit.MILLISECONDS);
        frontLauncherMotor.setCurrentPosistion(2000);
        rearLauncherMotor.setCurrentPosistion(2000);
        scoringMechanism.periodicTask();

        fakeTicker.advance(120, TimeUnit.MILLISECONDS);
        frontLauncherMotor.setCurrentPosistion(2000 + (int)((double)Launcher.LAUNCH_SPEED_ENC_SEC / 1000D * 120D));
        rearLauncherMotor.setCurrentPosistion(2000 + (int)((double)Launcher.LAUNCH_SPEED_ENC_SEC / 1000D * 120D));
        scoringMechanism.periodicTask();

        assertEquals(ScoringMechanism.LauncherReady.class, scoringMechanism.getCurrentState().getClass());

        // Launch a ring...
        launchTrigger.setPressed(true);
        scoringMechanism.periodicTask();
        assertEquals(ringFeedServo.getPosition(), Launcher.RING_FEEDER_FEEDING_POSITION, 0.05);

        // Let feeder return to park
        launchTrigger.setPressed(false);
        scoringMechanism.periodicTask();
        assertEquals(ringFeedServo.getPosition(), Launcher.RING_FEEDER_PARKED_POSITION, 0.05);

        // Stop the launcher
        causeButtonToRiseForNextPeriodTask(stopLaunchingButton);
        scoringMechanism.periodicTask();

        // We should be back at idle state if everything is working
        assertEquals(ScoringMechanism.IdleState.class, shouldBeIdleState.getClass());
        scoringMechanism.periodicTask();

        assertEquals(hopperPullDownServo.getPosition(), Launcher.HOPPER_PULLED_DOWN_POSITION, 0.05);
        assertEquals(ringFeedServo.getPosition(), Launcher.RING_FEEDER_PARKED_POSITION, 0.05);
    }

    // TODO: These are probably a good shared method somewhere
    private static void backwardsOnInput(FakeRangeInput input, float magnitude) {
        input.setCurrentPosition(1 * Math.abs(magnitude));
    }

    private static void forwardsOnInput(FakeRangeInput input, float magnitude) {
        input.setCurrentPosition(-1 * Math.abs(magnitude));
    }

    private void causeButtonToRiseForNextPeriodTask(FakeOnOffButton button) {
        button.setPressed(false);
        scoringMechanism.periodicTask();
        button.setPressed(true);
    }
}
