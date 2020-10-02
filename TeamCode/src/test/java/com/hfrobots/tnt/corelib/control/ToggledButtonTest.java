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

import com.ftc9929.corelib.control.ToggledButton;
import com.ftc9929.testing.fakes.control.FakeOnOffButton;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class ToggledButtonTest {
    private FakeOnOffButton upButton;

    private ToggledButton toggledButton;

    @Before
    public void setUp() {
        upButton = new FakeOnOffButton();
        toggledButton = new ToggledButton(upButton);
    }

    @Test
    public void happyPath() {
        // Remember, because the underlying button is debounced, we have to create an "edge"

        upButton.setPressed(false);
        Assert.assertFalse(toggledButton.isToggledTrue()); // throw away (ish)

        upButton.setPressed(true);
        Assert.assertTrue(toggledButton.isToggledTrue());

        upButton.setPressed(false);
        Assert.assertTrue(toggledButton.isToggledTrue());

        // Here's where the toggle happens
        upButton.setPressed(true);
        Assert.assertFalse(toggledButton.isToggledTrue());

        // FIXME: Since this is a toggle, what should isToggledTrue() return *now*?

    }
}
