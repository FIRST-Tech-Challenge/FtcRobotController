/*
Copyright (c) 2017 Robert Atkinson

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
package org.firstinspires.ftc.robotcore.external.hardware.camera;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.util.List;

@SuppressWarnings("WeakerAccess")
public interface CameraCharacteristics
{
    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    int[] getAndroidFormats();
    Size[] getSizes(int androidFormat);
    Size getDefaultSize(int androidFormat);
    long getMinFrameDuration(int androidFormat, Size size);
    int getMaxFramesPerSecond(int androidFormat, Size size);

    class CameraMode
    {
        public final int  androidFormat;
        public final Size size;
        public final long nsFrameDuration;
        public final int  fps;
        public final boolean isDefaultSize; // not used in equalitor

        public CameraMode(int androidFormat, Size size, long nsFrameDuration, boolean isDefaultSize)
        {
            this.androidFormat = androidFormat;
            this.size = size;
            this.nsFrameDuration = nsFrameDuration;
            this.fps = (int) (ElapsedTime.SECOND_IN_NANO / nsFrameDuration);
            this.isDefaultSize = isDefaultSize;
        }

        @Override public String toString()
        {
            return Misc.formatInvariant("CameraMode(format=%d %dx%d fps=%d)", androidFormat, size.getWidth(), size.getHeight(), fps);
        }

        @Override public boolean equals(Object o)
        {
            if (o instanceof CameraMode)
            {
                CameraMode them = (CameraMode)o;
                return androidFormat == them.androidFormat
                        && size.equals(them.size)
                        && nsFrameDuration == them.nsFrameDuration
                        && fps == them.fps;
            }
            else
                return super.equals(o);
        }

        @Override public int hashCode()
        {
            return Integer.valueOf(androidFormat).hashCode() ^ size.hashCode() ^ Integer.valueOf(fps).hashCode();
        }
    }

    List<CameraMode> getAllCameraModes();
}
