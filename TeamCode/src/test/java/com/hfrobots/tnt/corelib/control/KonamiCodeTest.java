/*
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)

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

package com.hfrobots.tnt.corelib.control;

import com.google.common.testing.FakeTicker;
import com.hfrobots.tnt.corelib.state.DelayState;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.fakes.FakeNinjaGamePad;
import com.hfrobots.tnt.fakes.FakeTelemetry;
import com.hfrobots.tnt.fakes.control.FakeOnOffButton;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class KonamiCodeTest {
    private FakeTicker ticker;

    private FakeNinjaGamePad gamePad;
    private FakeOnOffButton upButton;
    private FakeOnOffButton downButton;
    private FakeOnOffButton leftButton;
    private FakeOnOffButton rightButton;
    private FakeOnOffButton bButton;
    private FakeOnOffButton aButton;
    private KonamiCode konamiCode;
    private FakeTelemetry telemetry;

    @Before
    public void setUp() {

        ticker = new FakeTicker();
        ticker.setAutoIncrementStep(5, TimeUnit.MILLISECONDS);

        gamePad = new FakeNinjaGamePad();

        upButton = (FakeOnOffButton)gamePad.getDpadUp();
        downButton = (FakeOnOffButton)gamePad.getDpadDown();
        leftButton = (FakeOnOffButton)gamePad.getDpadLeft();
        rightButton = (FakeOnOffButton)gamePad.getDpadRight();
        bButton = (FakeOnOffButton)gamePad.getBButton();
        aButton = (FakeOnOffButton)gamePad.getAButton();

        telemetry = new FakeTelemetry();

        State terminalState = new DelayState("You've Arrived", telemetry, 0, TimeUnit.MILLISECONDS);

        konamiCode = new KonamiCode(gamePad, terminalState, ticker, telemetry);
    }

    @Test
    public void happyPath() {
        Assert.assertEquals("Konami - up_1?", konamiCode.getCurrentStateName());

        pressButtonCycleStateAndCheckName(upButton, "Konami - up_2?");

        pressButtonCycleStateAndCheckName(upButton, "Konami - down_1");

        pressButtonCycleStateAndCheckName(downButton, "Konami - down_2");

        pressButtonCycleStateAndCheckName(downButton, "Konami - left_1");

        pressButtonCycleStateAndCheckName(leftButton, "Konami - right_1");

        pressButtonCycleStateAndCheckName(rightButton, "Konami - left_2");

        pressButtonCycleStateAndCheckName(leftButton, "Konami - right_2");

        pressButtonCycleStateAndCheckName(rightButton, "Konami - b");

        pressButtonCycleStateAndCheckName(bButton, "Konami - a");

        pressButtonCycleStateAndCheckName(aButton, "You've Arrived");
    }

    private void pressButtonCycleStateAndCheckName(FakeOnOffButton upButton, String s) {
        pressAndCycleStateMachine(upButton);
        Assert.assertEquals(s, konamiCode.getCurrentStateName());
        konamiCode.periodicTask();
    }

    private void pressAndCycleStateMachine(FakeOnOffButton button) {
        button.setPressed(true);
        konamiCode.periodicTask();
        button.setPressed(false);
    }
}
