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

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

/**
 * An augmentation to {@link CameraIntrinsics} that helps support debugging
 * and parsing from XML.
 */
@SuppressWarnings("WeakerAccess")
public class CameraCalibration extends CameraIntrinsics implements Cloneable
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    /** These are not passed to native code */
    protected CameraCalibrationIdentity identity;
    protected Size size;
    protected boolean remove;
    protected final boolean isFake;

    @Override public String toString()
    {
        return Misc.formatInvariant("CameraCalibration(%s %dx%d f=%.3f,%.3f)", identity, size.getWidth(), size.getHeight(), focalLengthX, focalLengthY);
    }

    public CameraCalibrationIdentity getIdentity()
    {
        return identity;
    }
    public Size getSize()
    {
        return size;
    }
    public boolean getRemove()
    {
        return remove;
    }
    public boolean isFake()
    {
        return isFake;
    }

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public CameraCalibration(@NonNull CameraCalibrationIdentity identity, Size size, float focalLengthX, float focalLengthY, float principalPointX, float principalPointY, float[] distortionCoefficients, boolean remove, boolean isFake) throws RuntimeException
    {
        super(focalLengthX, focalLengthY, principalPointX, principalPointY, distortionCoefficients);
        this.identity = identity;
        this.size = size;
        this.remove = remove;
        this.isFake = isFake;
    }

    public CameraCalibration(@NonNull CameraCalibrationIdentity identity, int[] size, float[] focalLength, float[] principalPoint, float[] distortionCoefficients, boolean remove, boolean isFake) throws RuntimeException
    {
        this(identity, new Size(size[0], size[1]), focalLength[0], focalLength[1], principalPoint[0], principalPoint[1], distortionCoefficients, remove, isFake);
        if (size.length != 2) throw Misc.illegalArgumentException("frame size must be 2");
        if (principalPoint.length != 2) throw Misc.illegalArgumentException("principal point size must be 2");
        if (focalLength.length != 2) throw Misc.illegalArgumentException("focal length size must be 2");
        if (distortionCoefficients.length != 8) throw Misc.illegalArgumentException("distortion coefficients size must be 8");
    }

    public static CameraCalibration forUnavailable(CameraCalibrationIdentity calibrationIdentity, Size size)
    {
        if (calibrationIdentity==null)
        {
            calibrationIdentity = new org.firstinspires.ftc.robotcore.internal.camera.calibration.VendorProductCalibrationIdentity(0, 0);
        }
        return new CameraCalibration(calibrationIdentity, size, 0, 0, 0, 0, new float[8], false, true);
    }

    @SuppressWarnings({"unchecked"})
    protected CameraCalibration memberwiseClone()
    {
        try {
            return (CameraCalibration)super.clone();
        }
        catch (CloneNotSupportedException e)
        {
            throw new IllegalArgumentException(""); // ### throw AppUtil.getInstance().unreachable();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Access
    //----------------------------------------------------------------------------------------------

    /*
     * From: Dobrev, Niki <ndobrev@ptc.com>
     * Sent: Wednesday, May 23, 2018 4:57 AM
     *
     * Other option (in case the aspect ratio stays the same) is to just scale the principal point
     * and focal length values to match the resolution currently used before providing them to
     * Vuforia in the camera frame callback. Please keep in mind, that this is not optimal and it
     * is possible that it doesn't provide optimal results always, but from calibration effort point
     * of view is probably easier than doing calibrations for each supported resolution. Scaling
     * should work reasonably well in general case, if the camera is not doing anything strange when
     * switching capture resolutions.
     */
    public CameraCalibration scaledTo(Size newSize)
    {
        Assert.assertTrue(Misc.approximatelyEquals(getAspectRatio(newSize), getAspectRatio(size)));
        double factor = (double)(newSize.getWidth()) / (double)(size.getWidth());

        CameraCalibration result = memberwiseClone();
        result.size = newSize;
        result.focalLengthX *= factor;
        result.focalLengthY *= factor;
        result.principalPointX *= factor;
        result.principalPointY *= factor;
        return result;
    }

    public double getAspectRatio()
    {
        return getAspectRatio(size);
    }
    public double getDiagonal()
    {
        return getDiagonal(size);
    }

    public static double getDiagonal(Size size)
    {
        return Math.sqrt(size.getWidth() * size.getWidth() + size.getHeight() * size.getHeight());
    }
    protected static double getAspectRatio(Size size)
    {
        return (double)size.getWidth() / (double)size.getHeight();
    }

}
