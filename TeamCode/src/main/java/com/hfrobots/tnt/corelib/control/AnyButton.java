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

import com.google.common.collect.ImmutableSet;

import java.util.HashSet;
import java.util.Set;

public class AnyButton implements OnOffButton {
    private final Set<OnOffButton> allButtons;

    public AnyButton(OnOffButton... buttons /* shorthand for any amount of args */) {

        Set<OnOffButton> filteredButtons = new HashSet<>();

        for (OnOffButton button : buttons) {

          if (button != null) {
              filteredButtons.add(button);
          }
        }

        allButtons = ImmutableSet.copyOf(filteredButtons);
    }

    @Override
    public boolean isPressed() {
        for (OnOffButton button : allButtons) {
            if (button.isPressed()) {
                return true;
            }
        }

        return false;
    }
}