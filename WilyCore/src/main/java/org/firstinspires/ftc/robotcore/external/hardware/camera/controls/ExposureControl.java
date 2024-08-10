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

import org.firstinspires.ftc.robotcore.internal.collections.MutableReference;

import java.util.concurrent.TimeUnit;

public interface ExposureControl extends CameraControl
{
    //----------------------------------------------------------------------------------------------
    // Focus mode
    //----------------------------------------------------------------------------------------------

    enum Mode
    {
        Unknown,
        Auto,               // single trigger auto exposure
        ContinuousAuto,     // continuous auto exposure
        Manual,
        ShutterPriority,
        AperturePriority;

        public static Mode fromId(int id)
        {
            if (id >= 0 && id < values().length)
            {
                return values()[id];
            }
            return Unknown;
        }
    }

    /**
     * Get the current exposure mode of the camera
     * @return the current exposure mode of the camera
     */
    Mode getMode();

    /**
     * Set the exposure mode of the camera
     * @param mode the mode to enter
     * @return whether the operation was successful
     */
    boolean setMode(Mode mode);

    /**
     * Check whether the camera supports a given exposure mode
     * @param mode the mode in question
     * @return whether the mode in question is supported
     */
    boolean isModeSupported(Mode mode);

    //----------------------------------------------------------------------------------------------
    // Exposure duration
    //----------------------------------------------------------------------------------------------

    long unknownExposure = 0;

    /**
     * Get the shortest exposure time supported by the camera
     * @param resultUnit time units of your choosing
     * @return the shortest supported exposure time, or 0 if unavailable
     */
    long getMinExposure(TimeUnit resultUnit);

    /**
     * Get the longest exposure time supported by the camera
     * @param resultUnit time units of your choosing
     * @return the longest supported exposure time, or 0 if unavailable
     */
    long getMaxExposure(TimeUnit resultUnit);

    /**
     * Get the camera's current exposure time
     * @param resultUnit time units of your choosing
     * @return the camera's current exposure time, or 0 if unavailable
     */
    long getExposure(TimeUnit resultUnit);

    /** unknownExposure is returned if exposure unavailable */
    @Deprecated
    long getCachedExposure(final TimeUnit resultUnit, MutableReference<Boolean> refreshed, final long permittedStaleness, final TimeUnit permittedStalenessUnit);

    /**
     * Set the camera's exposure time. Only works if you're in manual mode.
     * @param duration exposure time
     * @param durationUnit time units of your choice
     * @return whether the operation succeeded
     */
    boolean setExposure(long duration, TimeUnit durationUnit);

    /**
     * Check whether your camera supports control over exposure
     * @return whether your camera supports control over exposure
     */
    boolean isExposureSupported();

    /**
     * Check whether AE priority is enabled
     * @return whether AE priority is enabled
     */
    boolean getAePriority();

    /**
     * Chooses whether the camera may vary the frame rate for exposure control reasons.
     * A priority value of false means the camera may not vary its frame rate. A value of true means the frame rate is variable
     * This setting has no effect outside of the auto and shutter_priority auto-exposure modes.
     * @return whether the operation was successful
     */
    boolean setAePriority(boolean priority);
}
