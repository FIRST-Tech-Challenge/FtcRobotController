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

import android.hardware.camera2.CameraCaptureSession;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;

@SuppressWarnings("WeakerAccess")
public interface Camera extends CameraControls
{
    @NonNull CameraName getCameraName();
    CameraCaptureRequest createCaptureRequest(int androidFormat, Size size, int fps) throws CameraException;
    @NonNull
    CameraCaptureSession createCaptureSession(Continuation<? extends CameraCaptureSession.StateCallback> continuation) throws CameraException;

    /**
     * Close the connection to this camera device as quickly as possible.
     *
     * <p>Immediately after this call, all calls to the camera device or active session interface
     * will throw a {@link IllegalStateException}, except for calls to close(). Once the device has
     * fully shut down, the {@link StateCallback#onClosed} callback will be called.</p>
     *
     * <p>Immediately after this call, besides the final {@link StateCallback#onClosed} calls, no
     * further callbacks from the device or the active session will occur, and any remaining
     * submitted capture requests will be discarded, and no success or failure
     * callbacks will be invoked.</p>
     */
    void close();

    /**
     * Returns another {@link Camera} on the same underlying device with an independent close()
     * effect. The analogy is that of dup()ing open file descriptors in Linux / Unix.
     */
    Camera dup();

    /**
     * Reasons for which a {@link Camera} might have failed to open. Note: this list might get
     * extended and elaborated in the future; do not assume that the present set of errors is
     * exhaustive. Rather, treat any errors you do not recognize as you do {@link #OtherFailure}.
     */
    enum OpenFailure
    {
        /** Never a real failure; handy in code */
        None,
        /** The open failed for an unspecified reason */
        OtherFailure,
        /** The type of camera is not supported in the current implementation */
        CameraTypeNotSupported,
        /** The camera is already open or security permissions prohibit us from opening it */
        InUseOrAccessDenied,
        /** The camera is currently disconnected */
        Disconnected,
        /** An internal failure (aka a 'bug') has occurred */
        InternalError,
    }

    /**
     * Reasons for which a {@link Camera} might have failed <em>after</em> opening. Note: this list
     * might get extended and elaborated in the future; do not assume that the present set of errors is
     * exhaustive. Rather, treat any errors you do not recognize as you do {@link #OtherError}.
     */
    enum Error
    {
        None,
        OtherError,
        Disconnected,
        Connected,
        StreamingRequestNotSupported,
        Timeout,
        InternalError,
    }

    interface StateCallback
    {
        /**
         * The method called when a camera device has finished successfully opening.
         *
         * <p>At this point, the camera device is ready to use, and {@link Camera#createCaptureSession}
         * can be called to set up a capture session.</p>
         *
         * <p>Important: once this method is invoked, callee is responsible for calling
         * {@link Camera#close()} when they are finished using the camera device.</p>
         *
         * @param camera the camera device that has become opened
         *
         * @see CameraManager#asyncOpenCameraAssumingPermission
         */
        void onOpened(@NonNull Camera camera);

        /**
         * A request to open a camera has failed.
         *
         * @param cameraName The identity of the camera that failed to open.
         *
         * @see CameraManager#asyncOpenCameraAssumingPermission
         */
        void onOpenFailed(@NonNull CameraName cameraName, @NonNull OpenFailure reason);

        /**
         * The method called when a camera device has been closed with
         * {@link Camera#close}.
         *
         * <p>Any attempt to call methods on this Camera in the
         * future will likely throw a {@link RuntimeException}.</p>
         *
         * @param camera the camera device that has become closed
         */
        void onClosed(@NonNull Camera camera);

        /**
         * The method called when a camera device has encountered a serious error.
         *
         * <p>This indicates a failure of the camera device or camera service in
         * some way. Any attempt to call methods on this Camera in the
         * future will likely throw a {@link CameraException}</p>
         *
         * <p>There may still be capture completion or camera stream callbacks
         * that will be called after this error is received.</p>
         *
         * <p>You should clean up the camera with {@link Camera#close} after
         * this happens. Further attempts at recovery are error-code specific.</p>
         *
         * @param camera The device reporting the error
         * @param error The error code
         */
        void onError(@NonNull Camera camera, Error error);
    }

    /** An implementation of StateCallback that does nothing by default */
    class StateCallbackDefault implements StateCallback
    {
        @Override public void onOpened(@NonNull Camera camera)
        {
            // ### closeCamera("StateCallbackDefault", camera);
        }
        @Override public void onOpenFailed(@NonNull CameraName cameraName, @NonNull OpenFailure failureReason)
        {
        }
        @Override public void onClosed(@NonNull Camera camera)
        {
        }
        @Override public void onError(@NonNull Camera camera, Error error)
        {
        }
    }

}
