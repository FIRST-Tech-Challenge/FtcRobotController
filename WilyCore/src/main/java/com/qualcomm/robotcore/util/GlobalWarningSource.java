/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.qualcomm.robotcore.util;

/**
 * Instances of this interface can be registered with RobotLog as dynamic
 * generators of robot warning messages.
 *
 * @see RobotLog#registerGlobalWarningSource(GlobalWarningSource)
 */
public interface GlobalWarningSource
{
    /**
     * Returns the current warning associated with this warning source. If the source currently
     * has no warning to contribute, it should return null or an empty string.
     *
     * @return the current warning associated with this warning source
     */
    String getGlobalWarning();

    /**
     * Returns true if the current warning (if any) is severe enough to warrant playing a warning
     * sound. If the source has no critical warning, this should return false. If getGlobalWarning()
     * returns null or a blank string, the value returned here will be ignored.
     */
    boolean shouldTriggerWarningSound();

    /**
     * Suppress or de-suppress the contributions of warnings by this source. If warnings are
     * suppressed, {@link #getGlobalWarning()} will always return an empty string. Internally,
     * a suppression count is maintained which is incremented if 'suppress' is true and decremented
     * if it is false. The initial value of the count is zero; suppression is in effect if the
     * count is greater than zero.
     *
     * @param suppress whether to suppress or de-suppress warnings associated with this source
     * @see #clearGlobalWarning()
     */
    void suppressGlobalWarning(boolean suppress);

    /**
     * Sets the current warning associated with this warning source, if permitted to do so
     * by the source. Sources may refuse for various reasons: perhaps they only ever produce
     * their warnings from internal state, or perhaps they don't accept another warning if a
     * first has already been set.
     *
     * @param warning the warning to be associated with this sources
     * @see #clearGlobalWarning()
     */
    void setGlobalWarning(String warning);

    /**
     * Clears any currently set warning (if permitted) for this source, and zeros
     * the sources suppression count.
     *
     * @see #setGlobalWarning(String)
     */
    void clearGlobalWarning();
}
