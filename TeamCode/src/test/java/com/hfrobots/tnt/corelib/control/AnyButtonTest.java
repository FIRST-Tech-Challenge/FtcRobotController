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

package com.hfrobots.tnt.corelib.control;

import com.ftc9929.corelib.control.AnyButton;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class AnyButtonTest {
    private FakeOnOffButton upButton;
    private FakeOnOffButton downButton;
    private FakeOnOffButton leftButton;
    private FakeOnOffButton rightButton;

    private AnyButton anyButton;

    @Before
    public void setUp() {
        upButton = new FakeOnOffButton();
        downButton = new FakeOnOffButton();
        leftButton = new FakeOnOffButton();
        rightButton = new FakeOnOffButton();

        anyButton = new AnyButton(upButton, downButton, leftButton, rightButton);
    }

    @Test
    public void happyPath() {
        pressButtonAndCheck(upButton);
        pressButtonAndCheck(downButton);
        pressButtonAndCheck(leftButton);
        pressButtonAndCheck(rightButton);
    }

    private void pressButtonAndCheck(FakeOnOffButton button) {
        button.setPressed(true);
        Assert.assertTrue(anyButton.isPressed());
    }

    @Test
    public void contentedPath() {
        upButton.setPressed(false);
        downButton.setPressed(false);
        leftButton.setPressed(false);
        rightButton.setPressed(false);

        Assert.assertFalse(anyButton.isPressed());
    }
}
