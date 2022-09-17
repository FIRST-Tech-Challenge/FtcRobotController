# Overview of Camera Interfaces

All camera interfaces implement the common OpenCvCamera interface. Extended functionality specific to each type of camera is exposed in individual interfaces:
 - OpenCvInternalCamera for the Android camera v1 API
   - Offers slightly lower overhead time compared to v2
   - Is **not** currently compatible with FTC Dash Camera stream (but the others are)
 - OpenCvInternalCamera2 for the Android camera v2 API
   - Offers much more fine grained control over the camera sensor compared ot v1, such as ISO, exposure, and focus control.
 - OpenCvWebcam for the FTC SDK's external webcam API

# Overview of Program Flow for Initializing a Camera

### 1. Using a live viewport (or not)

Before you can obtain a camera instance from the camera factory, you must decide whether or not you wish to have a live camera preview displayed on the Robot Controller screen. If so, you can obtain the view ID for the camera monitor container in exactly the same way as is done for Vuforia.
```
int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
```
If not, skip to section 2

Note that even if you choose to use a live viewport, you can programmatically pause it to reduce CPU/battery load using `camera.pauseViewport();` and `webcam.resumeViewport();`

### 2. Creating a Camera Instance Using the Camera Factory

All types of supported cameras are created by invoking the `OpenCvCameraFactory.getInstance().createXYZ(...)` methods.

 - To create an `OpenCvInternalCamera` instance:

 The first parameter specifies the desired camera (FRONT or BACK) and the second (optional) parameter specifies the view ID in which to insert the live preview. If you want to use a live preview, see section 1. If not, simply omit the second parameter.
 ```java
 // With live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

 // Without live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
 ```
 - To create an `OpenCvInternalCamera2` instance:

 The first parameter specifies the desired camera (FRONT or BACK) and the second (optional) parameter specifies the view ID in which to insert the live preview. If you want to use a live preview, see section 1. If not, simply omit the second parameter.
 ```java
 // With live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

 // Without live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK);
 ```

 - To create an `OpenCvWebcam` instance:

 The first parameter specifies the `WebcamName` of the webcam you wish to use. and the second (optional) parameter specifies the view ID in which to insert the live preview. If you want to use a live preview, see section 1. If not, simply omit the second parameter.

 Note: a `WebcamName` can be obtained from the `hardwareMap` like so:
 ```java
 WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE")
 ```

 Once you've obtained the `WebcamName`, you can proceed to using the camera factory:
 ```java
 // With live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

 // Without a live preview
 OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
 ```

### 3. Opening the Camera Device

Now that you've obtained an `OpenCvCamera` instance from the camera factory, the next step is to open the connection to the camera device. There are two methods for doing this:
 - Asynchronously (recommended).

   When opening asynchronously, your thread is not blocked. Instead, you provide a callback in order to be notified when the opening process has been completed. Usually it is in this callback that you'll want to start streaming from the camera (see section 4)

     ```java
     camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
     {
         @Override
         public void onOpened()
         {
             // Usually this is where you'll want to start streaming from the camera (see section 4)
         }
         @Override
         public void onError(int errorCode)
         {
            /*
            * This will be called if the camera could not be opened
            */
         }
     });
     ```
 - Synchronously (not recommended)

 When opening synchronously, your thread is blocked until the operation is complete.

 ```java
 camera.openCameraDevice();
 ```

### 4. Starting a Streaming Session

After opening the camera, you can start a streaming session by calling `camera.startStreaming(...)`. The first two parameters are the desired width and height of the image stream.

> NOTE: If you specify a resolution which the camera does not support, the program will crash and display an error message will tells you which resolutions ARE supported.
> Commonly supported resolutions include:
>  - 320x240
>  - 640x480
>  - 1280x720
>  - 1920x1080

The third parameter specifies the orientation the camera is being used in. So for instance, if you have a phone mounted in portrait on your robot, or a webcam used in its normal orientation, use `UPRIGHT`. But if you have the phone mounted in landscape on your robot, use `SIDEWAYS_LEFT` or `SIDEWAYS_RIGHT` depending on the specific way you've mounted it. `SIDEWAYS_LEFT` refers to the orientation where the screen is facing toward you in landscape mode and the USB port is facing to the right.
```java
camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
```

> NOTE: If you are using a an internal phone camera in a streaming orientation which does not match the current activity orientation, you will find that the live preview is rendered 90 degrees out from how it "should be". This does NOT impact how frames are delivered to your code. However, if you wish to correct for this, you can call
> ```java
> camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
> ```
> Keep in mind this setting is not applicable when using a webcam. It's only needed when the camera orientation is physically tied to the device orientation, which is the case with internal cameras.
>
> NOTE: Irrespective of whether you're using the `OPTIMIZE_VIEW` policy (but especially if you are), you may want to enable GPU-accelerated rendering for the viewport. This can be done with:
> ```java
> // NOTE: this must be called *before* you call startStreaming(...)
> camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
> ```
> Using GPU acceleration allows for faster render time to the display, which can reduce CPU load and make the preview smoother. It's especially helpful when using `OPTIMIZE_VIEW` because of the rotation that it may perform under the hood.
> However, using GPU acceleration has been observed to occasionally cause crashes in libgles.so / libutils.so on some devices, if the activity orientation is changed. (i.e. you rotate the device w/o rotation lock on) while a streaming session is in flight. Caveat emptor.

### 5. Attaching a Pipeline

A streaming session won't do you much good without any OpenCV processing running on it! [To learn about creating a pipeline, please click here](pipelines_overview.md).
You can attach a pipeline to the camera as follows:
```java
camera.setPipeline(yourPipeline);
```

Pipelines may be set at any point during the camera lifecycle. You can even change pipelines while a streaming session is in flight.

### 6. Closing the Camera / Stopping a Streaming Session

Most of the time there is no need to manually close the camera, as it will be automatically closed for you at the end of your OpMode. However, the `OpenCvCamera` interface does provide methods for closing the camera manually. Please refer to the [JavaDocs](https://javadoc.io/doc/org.openftc/easyopencv/latest/org/openftc/easyopencv/OpenCvCamera.html) for information about these methods.

# Using the Driver Station Camera Preview Feature

For the Control Hub, the DS camera preview feature may prove very helpful for you since it can show the output of your pipeline without requiring an HDMI monitor or use of `scrcpy`.

To use this feature, simply make sure that you've started a streaming session during the OpMode initialization. When running your OpMode, do **NOT** press start. Only press INIT. While the OpMode is in the INIT phase, you can open the overflow menu on the DriverStation and select the "Camera Stream" option. This provides a tap-to-refresh view of the pipeline output.
