package org.firstinspires.ftc.teamcode.auto.cv.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.HardwareNew;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="OP MODE for Pipeline")
public class OpenOpMode extends LinearOpMode {
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    Pipeline detector = new Pipeline(width);
    OpenCvCamera Camera;
    HardwareNew robot = new HardwareNew();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // robot logic...

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        Camera.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        Camera.setPipeline(detector);
        // Remember to change the camera rotation
        Camera.startStreaming(width, height, OpenCvCameraRotation.UPSIDE_DOWN);

        //...

        waitForStart();

        while(opModeIsActive())
        {
            Pipeline.ShippingElemLocation location = detector.getLocation();
            if (location == null) {
                // It has not been set by the opencv library as yet.
                continue;
            }
            if (location != Pipeline.ShippingElemLocation.NONE) {
                if (location == Pipeline.ShippingElemLocation.LEFT)
                {

                    telemetry.addLine("Going In");

                } else if (location == Pipeline.ShippingElemLocation.RIGHT) {
                    telemetry.addLine("Going Out");

                }
            } else if (location == Pipeline.ShippingElemLocation.NONE && opModeIsActive()){
                robot.turnLeft(5, 0.5);
                telemetry.addLine("Haven't found Green");

            }

            telemetry.update();
        }

        // more robot logic...
    }

}