/**
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
 **/

package com.hfrobots.tnt.corelib.control;

import com.hfrobots.tnt.fakes.control.FakeRangeInput;

import junit.framework.TestCase;

public class LowPassFilteredRangeInputTest extends TestCase {
    private FakeRangeInput fakeInput;
    private LowPassFilteredRangeInput lowPassFilteredRangeInput;

    @Override
    protected void setUp() throws Exception {
        super.setUp();

        fakeInput = new FakeRangeInput();
        lowPassFilteredRangeInput = new LowPassFilteredRangeInput(fakeInput, 0.12F);
    }

    public void testFiltered() {
        fakeInput.setCurrentPosition(0);
        assertEquals(0F, lowPassFilteredRangeInput.getPosition());
        fakeInput.setCurrentPosition(1.0F);
        float filteredPosition = lowPassFilteredRangeInput.getPosition();
        assertTrue("Value " + filteredPosition + " does not appear to be filtered",
                filteredPosition != 1.0F);
    }
}