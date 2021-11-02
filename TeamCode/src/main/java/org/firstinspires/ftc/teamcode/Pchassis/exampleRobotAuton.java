// modified from https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/firstinspires/ftc/teamcode/InternalCamera1Example.java

package org.firstinspires.ftc.teamcode.Pchassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.BlueVisionRGB;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Example Marker Detection")
public class exampleRobotAuton extends LinearOpMode
{
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode()
    {

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using. In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing camera.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        BlueVisionRGB p = new BlueVisionRGB(telemetry);
        phoneCam.setPipeline(p);

        // Open the connection to the camera device.

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while(opModeIsActive()) {
            boolean[] pos = p.getPositions();
            if(pos != null) {
                if(pos[0] && pos[1] && !pos[2]) {
                    telemetry.addLine("Marker on right");
                } else if(pos[0] && pos[2] && !pos[1]) {
                    telemetry.addLine("Marker in middle");
                } else if(pos[1] && pos[2] && !pos[0]) {
                    telemetry.addLine("Marker on left");
                } else {
                    telemetry.addLine("Default case");
                }
                telemetry.update();
            }

        }
    }
}