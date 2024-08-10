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

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.List;
import java.util.concurrent.TimeUnit;

public interface CameraManager
{
    //----------------------------------------------------------------------------------------------
    // Camera Names
    //----------------------------------------------------------------------------------------------

    /**
     * Return the list of currently connected camera devices which are USB webcams.
     * Note: this list may contain duplicates if more than once instance of a serialnumberless
     * webcam is attached.
     *
     * @return The list of currently connected USB webcams.
     */
    List<WebcamName> getAllWebcams();

    /**
     * Returns a camera name indicating a particular {@link BuiltinCameraDirection}.
     * Using this method, both camera directions and webcam names can be indicated with a
     * uniform name representation.
     *
     * @return a camera name indicating a particular {@link BuiltinCameraDirection}
     */
    CameraName nameFromCameraDirection(BuiltinCameraDirection cameraDirection);

    /**
     * Returns a {@link CameraName} which is guaranteed never to represent that of an actual
     * camera. This can be useful in situations where a value distinguished from null and
     * actual camera names is desired.
     *
     * @return a {@link CameraName} which is guaranteed never to represent that of an actual camera
     */
    CameraName nameForUnknownCamera();
    SwitchableCameraName nameForSwitchableCamera(CameraName... cameraNames);

    //----------------------------------------------------------------------------------------------
    // Opening cameras
    //----------------------------------------------------------------------------------------------

    Camera requestPermissionAndOpenCamera(Deadline deadline, CameraName cameraName, @Nullable Continuation<? extends Camera.StateCallback> continuation);
    void asyncOpenCameraAssumingPermission(@NonNull final CameraName cameraName,
                                           @NonNull final Continuation<? extends Camera.StateCallback> continuation,
                                           long reopenDuration, TimeUnit reopenTimeUnit);
}
