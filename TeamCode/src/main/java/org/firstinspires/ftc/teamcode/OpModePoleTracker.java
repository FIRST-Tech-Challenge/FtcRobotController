package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Capable;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevelString;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getMinRectHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getMinRectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getPercentColor;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getRectHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getRectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getXResolution;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getYResolution;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.percentColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="PoleTracker", group="A")
public class OpModePoleTracker extends LinearOpMode {
    String level = "one";
    int levelCounter = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        PipePoleTracker pipePoleTracker = new PipePoleTracker(level);
        camera.setPipeline(pipePoleTracker);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);


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


        while(opModeIsActive()){


            if(gamepad2.a && getLevel2Capable()){
                levelCounter = 2;
            }

            if(levelCounter == 2){
                level = "two";
            }

            if(levelCounter == 1){
                level = "one";
            }

            telemetry.addLine("Current Level: " + getLevelString());
            telemetry.addLine("X_resolution: " + getXResolution());
            telemetry.addLine("Y_resolution: " + getYResolution());
            telemetry.addLine("Level 2 Capable?: " + getLevel2Capable());
            telemetry.addLine("# of Nonzeros: " + getPercentColor());
            telemetry.addLine("Focus Rectangle Width " + getRectWidth());
            telemetry.addLine("Focus Rectangle Height: " + getRectHeight());
            telemetry.addLine("Rectangle Min Width: " + getMinRectWidth());
            telemetry.addLine("Rectangle Min Height: " + getMinRectHeight());

            telemetry.update();

            pipePoleTracker = new PipePoleTracker(level);
            camera.setPipeline(pipePoleTracker);
        }
    }
}
