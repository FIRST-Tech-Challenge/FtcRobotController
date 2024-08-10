/*
Copyright (c) 2018 Robert Atkinson

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
package org.firstinspires.ftc.robotcore.external.hardware.camera.controls;

public interface FocusControl extends CameraControl
{
    //----------------------------------------------------------------------------------------------
    // Focus mode
    //----------------------------------------------------------------------------------------------

    enum Mode
    {
        Unknown,
        Auto,               // single trigger auto focus
        ContinuousAuto,     // continuous auto focus
        Macro,              //
        Infinity,           //
        Fixed;              // Fixed focus that can't be adjusted

        public static FocusControl.Mode fromId(int index)
        {
            if (index >= 0 && index < values().length)
            {
                return values()[index];
            }
            return Unknown;
        }
    }

    /**
     * Get the current focus mode of the camera
     * @return the current focus mode of the camera
     */
    Mode getMode();


    /**
     * Set the focus mode of the camera
     * @param mode the mode to enter
     * @return whether the operation was successful
     */
    boolean setMode(Mode mode);

    /**
     * Check whether the camera supports a given focus mode
     * @param mode the mode in question
     * @return whether the mode in question is supported
     */
    boolean isModeSupported(Mode mode);

    //----------------------------------------------------------------------------------------------
    // Focus length: units are in mm
    //----------------------------------------------------------------------------------------------

    double unknownFocusLength = -1.0;

    /**
     * Get the minimum focal distance supported by the camera
     * @return the minimum focal distance supported, or 0 if unavailable
     */
    double getMinFocusLength();

    /**
     * Get the maximum focal distance supported by the camera
     * @return the maximum focal distance supported, or 0 if unavailable
     */
    double getMaxFocusLength();

    /**
     * Get the camera's current focus length
     * @return the camera's current focus length, or 0 if unavailable
     */
    double getFocusLength();

    /**
     * Set the camera's focus length. Only works if you're in fixed mode.
     * @param focusLength the desired focus length
     * @return whether the operation succeeded
     */
    boolean setFocusLength(double focusLength);

    /**
     * Check whether your camera supports control over focus
     * @return whether your camera supports control over focus
     */
    boolean isFocusLengthSupported();
}
