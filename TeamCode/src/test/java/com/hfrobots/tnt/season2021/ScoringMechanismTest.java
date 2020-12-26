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

import com.ftc9929.corelib.state.State;
import com.ftc9929.testing.fakes.FakeTelemetry;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;
import com.ftc9929.testing.fakes.control.FakeRangeInput;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.drive.FakeServo;
import com.ftc9929.testing.fakes.util.FakeHardwareMapFactory;
import com.google.common.testing.FakeTicker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class ScoringMechanismTest {
    private FakeDcMotorEx intakeMotor;

    private FakeDcMotorEx frontLauncherMotor;

    private FakeDcMotorEx rearLauncherMotor;

    private FakeServo ringFeedServo;

    @Before
    public void setup() {
        intakeMotor = (FakeDcMotorEx) UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "intakeMotor");
        frontLauncherMotor = (FakeDcMotorEx) UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "frontLauncherMotor");
        rearLauncherMotor = (FakeDcMotorEx) UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "rearLauncherMotor");

        ringFeedServo = (FakeServo) UltimateGoalTestConstants.HARDWARE_MAP.get(Servo.class, "ringFeedServo");
    }

    @Test
    public void testIntakeToFiring() {
        FakeTicker fakeTicker = new FakeTicker();
        FakeTelemetry fakeTelemetry = new FakeTelemetry();
        FakeRangeInput intakeVelocity = new FakeRangeInput();
        FakeOnOffButton launchTrigger = new FakeOnOffButton();

        ScoringMechanism scoringMechanism = ScoringMechanism.builder()
                .hardwareMap(UltimateGoalTestConstants.HARDWARE_MAP)
                .intakeVelocity(intakeVelocity)
                .launchTrigger(launchTrigger)
                .telemetry(fakeTelemetry).ticker(fakeTicker).build();

        State shouldBeIdleState = scoringMechanism.getCurrentState();
        assertEquals(ScoringMechanism.IdleState.class, shouldBeIdleState.getClass());

        backwardsOnInput(intakeVelocity, .5F);

        scoringMechanism.periodicTask();

        State shouldBeIntaking = scoringMechanism.getCurrentState();
        assertEquals(ScoringMechanism.IntakeMoving.class, shouldBeIntaking.getClass());
    }

    // TODO: These are probably a good shared method somewhere
    private static void backwardsOnInput(FakeRangeInput input, float magnitude) {
        input.setCurrentPosition(1 * Math.abs(magnitude));
    }

    private static void forwardsOnInput(FakeRangeInput input, float magnitude) {
        input.setCurrentPosition(-1 * Math.abs(magnitude));
    }

}
