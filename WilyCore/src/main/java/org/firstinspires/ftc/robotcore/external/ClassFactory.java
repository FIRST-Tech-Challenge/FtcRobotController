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
package org.firstinspires.ftc.robotcore.external;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.FrameGenerator;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodParameters;
import java.util.List;

/**
 * {@link ClassFactory} provides a means by which various objects in the SDK may be logically
 * instantiated without exposing their external class identities to user's programs.
 */
@SuppressWarnings("WeakerAccess")
public abstract class ClassFactory
{
    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public static ClassFactory getInstance()
    {
        return InstanceHolder.theInstance;
    }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    /**
     * createTFObjectDetector returns an instance of the TensorFlow object detector engine
     * configured with the indicated set of parameters and using the given FrameGenerator to
     * supply the camera frames
     *
     * @param parameters the parameters used to configure the instance of the engine
     * @param frameGenerator the FrameGenerator that will be used to supply camera frames
     * @return an instance of the TensorFlow object detector engine.
     *
     * @see TFObjectDetector
     */
    public abstract TFObjectDetector createTFObjectDetector(TfodParameters parameters,
                                                            FrameGenerator frameGenerator);

    /**
     * Returns a {@link CameraManager} which can be used to access the USB webcams
     * attached to the robot controller.
     * @see CameraManager
     */
    public abstract CameraManager getCameraManager();

    /**
     * Returns a name of a virtual camera comprised of all the webcams configured in the given
     * hardware map.
     */
    public static CameraName createSwitchableCameraNameForAllWebcams(HardwareMap hardwareMap)
    {
        List<WebcamName> list = hardwareMap.getAll(WebcamName.class);
        CameraName[] allWebcams = list.toArray(new CameraName[list.size()]);
        return getInstance().getCameraManager().nameForSwitchableCamera(allWebcams);
    }

    //----------------------------------------------------------------------------------------------
    // Internal
    //----------------------------------------------------------------------------------------------

    protected static class InstanceHolder
    {
        public static ClassFactory theInstance = null;
    }
}
