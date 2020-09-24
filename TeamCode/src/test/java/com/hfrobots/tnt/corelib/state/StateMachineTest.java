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

package com.hfrobots.tnt.corelib.state;

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;

import junit.framework.TestCase;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class StateMachineTest extends TestCase {
    TestOnOffButton goButton;
    TestOnOffButton goBackButton;
    TestOnOffButton doOverButton;

    StateMachine stateMachine;
    TestTelemetry testTelemetry;

    @Override
    protected void setUp() throws Exception {
        super.setUp();

        goButton = new TestOnOffButton();
        goBackButton = new TestOnOffButton();
        doOverButton = new TestOnOffButton();
        testTelemetry = new TestTelemetry();

        stateMachine = new StateMachine(testTelemetry);
        stateMachine.setGoBackButton(new DebouncedButton(goBackButton));
        stateMachine.setGoButton(new DebouncedButton(goButton));
        stateMachine.setDoOverButton(new DebouncedButton(doOverButton));
    }

    public void testNormalOperation() {
        TestState startState = new TestState("start", null);
        TestState state2 = new TestState("state2", null);
        startState.setNextState(state2);
        TestState state3 = new TestState("state3", null);
        state2.setNextState(state3);

        stateMachine.setFirstState(startState);
        stateMachine.addNewState(state2);
        stateMachine.addNewState(state3);

        assertEquals("start", stateMachine.getCurrentStateName());

        stateMachine.doOneStateLoop();
        assertEquals("> state state2", testTelemetry.telemetryData.get("00"));
        assertEquals("state2", stateMachine.getCurrentStateName());

        stateMachine.doOneStateLoop();
        assertEquals("> state state3", testTelemetry.telemetryData.get("00"));
        assertEquals("state3", stateMachine.getCurrentStateName());
    }

    public void testDebugging() throws Exception {

        TestState throwAwayState = new TestState("Throw away", testTelemetry);

        TestState stateA = new TestState("State A", testTelemetry);
        throwAwayState.setNextState(stateA);

        TestState stateB = new TestState("State B", testTelemetry);
        stateA.setNextState(stateB);

        stateMachine.setFirstState(throwAwayState);
        stateMachine.addNewState(stateA);
        stateMachine.addNewState(stateB);

        stateMachine.startDebugging();
        stateMachine.doOneStateLoop(); // should go from throw-away to stateA
        System.out.println(testTelemetry.telemetryData.get("00"));
        assertEquals(1, throwAwayState.executionCount);
        assertEquals(0, stateA.executionCount);

        for (int i = 0; i < 10; i++) {
            stateMachine.doOneStateLoop(); // we should actually be configurable
            assertEquals(0, stateA.executionCount);
            assertEquals(i + 1, stateA.configuredCallCount);
        }

        goButton.pressed = true;
        stateMachine.doOneStateLoop(); // catch the "go" button being pressed
        goButton.pressed = false;
        stateMachine.doOneStateLoop(); // we should run state "a" and pause at state "b"
        System.out.println(testTelemetry.telemetryData.get("00"));

        goBackButton.pressed = true;
        stateMachine.doOneStateLoop(); // catch the "go back" button being pressed
        goBackButton.pressed = false;
        System.out.println(testTelemetry.telemetryData.get("00"));
    }

    class TestState extends State {
        private int executionCount = 0;

        private int configuredCallCount = 0;

        private int configurableParameter = 0;

        public TestState(String name, Telemetry telemetry) {
            super(name, telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            executionCount++;
            return nextState;
        }

        @Override
        public void resetToStart() {
            executionCount = 0;
        }

        @Override
        public void liveConfigure(DebouncedGamepadButtons buttons) {
            configuredCallCount++;

            //if (buttons.getaButton().getRise()) {
              //  configurableParameter++;
            //}
        }
    }

    class TestOnOffButton implements OnOffButton {
        boolean pressed = false;

        @Override
        public boolean isPressed() {
            return pressed;
        }
    }

    class TestTelemetry implements Telemetry {
        Map<String, String> telemetryData = new HashMap<>();

        @Override
        public Item addData(String caption, String format, Object... args) {
            telemetryData.put(caption, String.format(format, args));

            return null;
        }

        @Override
        public Item addData(String caption, Object value) {
            telemetryData.put(caption, value.toString());

            return null;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return null;
        }

        @Override
        public boolean removeItem(Item item) {
            return false;
        }

        @Override
        public void clear() {

        }

        @Override
        public void clearAll() {

        }

        @Override
        public Object addAction(Runnable action) {
            return null;
        }

        @Override
        public boolean removeAction(Object token) {
            return false;
        }

        @Override
        public void speak(String text) {

        }

        @Override
        public void speak(String text, String languageCode, String countryCode) {

        }

        @Override
        public boolean update() {
            return false;
        }

        @Override
        public Line addLine() {
            return null;
        }

        @Override
        public Line addLine(String lineCaption) {
            return null;
        }

        @Override
        public boolean removeLine(Line line) {
            return false;
        }

        @Override
        public boolean isAutoClear() {
            return false;
        }

        @Override
        public void setAutoClear(boolean autoClear) {

        }

        @Override
        public int getMsTransmissionInterval() {
            return 0;
        }

        @Override
        public void setMsTransmissionInterval(int msTransmissionInterval) {

        }

        @Override
        public String getItemSeparator() {
            return null;
        }

        @Override
        public void setItemSeparator(String itemSeparator) {

        }

        @Override
        public String getCaptionValueSeparator() {
            return null;
        }

        @Override
        public void setCaptionValueSeparator(String captionValueSeparator) {

        }

        @Override
        public void setDisplayFormat(DisplayFormat displayFormat) {

        }

        @Override
        public Log log() {
            return null;
        }
    }
}