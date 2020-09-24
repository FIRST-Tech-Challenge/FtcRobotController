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

import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.state.ButtonPressWithTimeoutState;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StateMachine;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.NonNull;

public class KonamiCode {
    private final StateMachine stateMachine;

    public KonamiCode(@NonNull NinjaGamePad gamePad,
                      @NonNull State whenCodeValidState,
                      @NonNull Ticker ticker,
                      @NonNull Telemetry telemetry) {
        stateMachine = new StateMachine(null /* hide the konami code from telemetry */);

        DebouncedButton upButton = new DebouncedButton(gamePad.getDpadUp());
        DebouncedButton downButton = new DebouncedButton(gamePad.getDpadDown());
        DebouncedButton leftButton = new DebouncedButton(gamePad.getDpadLeft());
        DebouncedButton rightButton = new DebouncedButton(gamePad.getDpadRight());
        DebouncedButton bButton = new DebouncedButton(gamePad.getBButton());
        DebouncedButton aButton = new DebouncedButton(gamePad.getAButton());

        ButtonPressWithTimeoutState firstUp = new ButtonPressWithTimeoutState("Konami - up_1?",
                upButton,
                null, // FIXME: There's a circular dependency here....
                telemetry,
                ticker, 1750);

        firstUp.setWhenTimedOutState(firstUp);

        State secondUp = new ButtonPressWithTimeoutState("Konami - up_2?",
                upButton,
                firstUp,
                telemetry,
                ticker, 1750);

        State firstDown = new ButtonPressWithTimeoutState("Konami - down_1",
                downButton,
                secondUp,
                telemetry,
                ticker, 1750);

        State secondDown = new ButtonPressWithTimeoutState("Konami - down_2",
                downButton,
                firstDown,
                telemetry,
                ticker, 1750);

        State firstLeft = new ButtonPressWithTimeoutState("Konami - left_1",
                leftButton,
                secondDown,
                telemetry,
                ticker, 1750);

        State firstRight = new ButtonPressWithTimeoutState("Konami - right_1",
                rightButton,
                firstLeft,
                telemetry,
                ticker, 1750);

        State secondLeft = new ButtonPressWithTimeoutState("Konami - left_2",
                leftButton,
                firstRight,
                telemetry,
                ticker, 1750);

        State secondRight = new ButtonPressWithTimeoutState("Konami - right_2",
                rightButton,
                secondLeft,
                telemetry,
                ticker, 1750);
        State b = new ButtonPressWithTimeoutState("Konami - b",
                bButton,
                secondRight,
                telemetry,
                ticker, 1750);

        State a = new ButtonPressWithTimeoutState("Konami - a",
                aButton,
                b,
                telemetry,
                ticker, 1750);

        stateMachine.addSequential(firstUp);
        stateMachine.addSequential(secondUp);
        stateMachine.addSequential(firstDown);
        stateMachine.addSequential(secondDown);
        stateMachine.addSequential(firstLeft);
        stateMachine.addSequential(firstRight);
        stateMachine.addSequential(secondLeft);
        stateMachine.addSequential(secondRight);
        stateMachine.addSequential(b);
        stateMachine.addSequential(a);
        stateMachine.addSequential(whenCodeValidState);
    }

    public void periodicTask() {
        stateMachine.doOneStateLoop();
    }

    public String getCurrentStateName() {
        return stateMachine.getCurrentStateName();
    }
}