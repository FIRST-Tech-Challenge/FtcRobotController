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
package org.firstinspires.ftc.robotcore.internal.camera.calibration;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.Arrays;

/**
 * Provides basic information regarding some characteristics which are built-in to / intrinsic
 * to a particular camera model.
 *
 * https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
 * https://docs.opencv.org/3.0-beta/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
 * https://www.mathworks.com/help/vision/camera-calibration.html
 *
 * @see CameraCalibration
 */
@SuppressWarnings("WeakerAccess")
public class CameraIntrinsics
{
    /** Focal length x-component. 0.f if not available. */
    public float focalLengthX;

    /** Focal length y-component. 0.f if not available. */
    public float focalLengthY;

    /** Principal point x-component. 0.f if not available. */
    public float principalPointX;

    /** Principal point y-component. 0.f if not available. */
    public float principalPointY;

    /**
     * An 8 element array of distortion coefficients.
     * Array should be filled in the following order (r: radial, t:tangential):
     *    [r0, r1, t0, t1, r2, r3, r4, r5]
     * Values that are not available should be set to 0.f.
     *
     * Yes, the parameter order seems odd, but it is correct.
     */
    @NonNull public final float[] distortionCoefficients;

    public CameraIntrinsics(float focalLengthX, float focalLengthY, float principalPointX, float principalPointY, float[] distortionCoefficients) throws RuntimeException
    {
        if (distortionCoefficients != null && distortionCoefficients.length == 8)
        {
            this.focalLengthX = focalLengthX;
            this.focalLengthY = focalLengthY;
            this.principalPointX = principalPointX;
            this.principalPointY = principalPointY;
            this.distortionCoefficients = Arrays.copyOf(distortionCoefficients, distortionCoefficients.length);
        }
        else
            throw Misc.illegalArgumentException("distortionCoefficients must have length 8");
    }

    public float[] toArray()
    {
        float[] result = new float[12];
        result[0] = focalLengthX;
        result[1] = focalLengthY;
        result[2] = principalPointX;
        result[3] = principalPointY;
        System.arraycopy(distortionCoefficients,  0, result, 4, 8);
        return result;
    }

    public boolean isDegenerate()
    {
        return focalLengthX==0 && focalLengthY==0 && principalPointX==0 && principalPointY==0
                && distortionCoefficients[0]==0
                && distortionCoefficients[1]==0
                && distortionCoefficients[2]==0
                && distortionCoefficients[3]==0
                && distortionCoefficients[4]==0
                && distortionCoefficients[5]==0
                && distortionCoefficients[6]==0
                && distortionCoefficients[7]==0;
    }

}
