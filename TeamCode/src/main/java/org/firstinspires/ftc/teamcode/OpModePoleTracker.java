package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxBL_X;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxBL_Y;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getHighestX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getHighestY;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel1Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Capable;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevelString;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLowestX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLowestY;
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
            telemetry.addLine("Level1 Assigment: " + getLevel1Assigment());
            telemetry.addLine("Level2 Assignment: " + getLevel2Assigment());
            telemetry.addLine("X_resolution: " + getXResolution());
            telemetry.addLine("Y_resolution: " + getYResolution());
            telemetry.addLine("Level 2 Capable?: " + getLevel2Capable());
            telemetry.addLine("# of Nonzeros: " + getPercentColor());
            telemetry.addLine("Focus Rectangle Width " + getRectWidth());
            telemetry.addLine("Focus Rectangle Height: " + getRectHeight());
            telemetry.addLine("Rectangle Min Width: " + getMinRectWidth());
            telemetry.addLine("Rectangle Min Height: " + getMinRectHeight());
            telemetry.addLine("Box Width: " + getBoxWidth());
            telemetry.addLine("Box Height: " + getBoxHeight());
            telemetry.addLine("LowestX: " + getLowestX());
            telemetry.addLine("HighestX: " + getHighestX());
            telemetry.addLine("LowestY: " + getLowestY());
            telemetry.addLine("HighestY: " + getHighestY());
            telemetry.addLine("Box_BL_x: " + getBoxBL_X());
            telemetry.addLine("Box_BL_y: " + getBoxBL_Y());





            telemetry.update();

            pipePoleTracker = new PipePoleTracker(level);
            camera.setPipeline(pipePoleTracker);
        }
    }
}
