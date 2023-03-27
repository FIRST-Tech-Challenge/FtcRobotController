package org.firstinspires.ftc.teamcode.opModes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * Description: [Fill in]
 * Hardware:
 *  [motor0] Unused
 *  [motor1] Unused
 *  [motor2] Unused
 *  [motor3] Unused
 *  [servo0] Unused
 *  [servo1] Unused
 *  [servo2] Unused
 *  [servo3] Unused
 */
@Disabled
@Autonomous(name="<OpMode name>", group="<OpMode group name>")
public class TestCustomSleeveCamera extends AutonomousLinearModeBase {
// Declare class members here
private ElapsedTime runtime = new ElapsedTime();

// EasyOpenCV stuff, shouldn't be used

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

    @Override
    public void run() {
        // Code to run once INIT is pressed
        waitForStart();
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
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//      camera.setPipeline("a");  //shove pipeline here
        // Code to run once PLAY is pressed
        runtime.reset();
        // Run until the driver presses STOP
        while (opModeIsActive()) {
        // Code to run in a loop
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        }
    }
}