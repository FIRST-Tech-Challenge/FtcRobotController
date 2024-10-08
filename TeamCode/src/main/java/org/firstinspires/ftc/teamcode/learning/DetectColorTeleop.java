package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.centerstage.SpikeMark;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * ColorDetectionOpMode
 *
 * This OpMode uses a webcam to detect the prominent color (yellow, blue, or red) in front of the camera.
 * Detection results are displayed via telemetry and visually annotated on the camera feed.
 * Only one color will be flagged as detected based on the area occupied by each color in the frame.
 */
@TeleOp
public class DetectColorTeleop extends LinearOpMode {

    //Webcam variables
    OpenCvCamera webcam; //Represents the webcam
    ColorDetectionPipeline pipeline; // Our custom pipelinefor color detection

    /**
     * runOpMode
     *
     * This is the main method tht run when the OpMode is selected and started.
     * It initializes the webcam and pipeline, then continuously checks for the most prominent color.
     */
    @Override
    public void runOpMode() {
        // Initialize the pipeline
        pipeline = new ColorDetectionPipeline();
        pipeline.myOpMode = this;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(pipeline);


        //Get the camera monitor view ID from the hardwareMap
        //This ID is used to desplay the camera feed on the driver station phone
        //     int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        //             "cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());

        //Initalize the webcam using EasyOpenCV
        //"Webcam1"should match the name you assigned in the Robot Controller app
        //     webcam = OpenCvCameraFactory.getInstance().createWebcam(
        //             hardwareMap.get(WebcamName.class,"Webcam 1"),cameraMonitorViewId);

        //Set the pipeline to proccess each frame
        //    webcam.setPipeline(pipeline);

        //Open the camera device asynchronously to avoid blocked the main thread
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Start streaming with anresolution of 640x480 and upright rotation
                //Adjust resolution as needed; higher resolutions require more processing power
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){
                //Handle camera opening errors
                telemetry.addData("Camera Error", "Failed to open camera with error code:"+errorCode);
                telemetry.update();
            }
        });

        //Display results via telemetry
        telemetry.addLine("Camera initialized. Waiting for start...");
        telemetry.update();

        //wait for the game to start (driver presses PLAY)
        waitForStart();

        //Main loop
        while (opModeIsActive()) {
            //Retrive the most prominent deteced color from the pipeline
            ColorDetectionPipeline.DetectedColor currentColor =pipeline.detectedColor;

            //Displayed results via telemetry
            telemetry.addLine("most Prominent Color Detected:");
            switch (currentColor) {
                case YELLOW:
                    telemetry.addData("Color", "Yellow");
                    break;
                case BLUE:
                    telemetry.addData("Color", "Blue");
                    break;
                case RED:
                    telemetry.addData("Color","Red");
                    break;
                case NONE:
                    telemetry.addData("color", "None");
                    break;
            }
            telemetry.update();
            sleep(50);

            //Opional: Add actions based on detections

            //For exsample, activate a mechanism when a specific color is detected
            /*
            if (currentColorDetectionPipeline.DetectedColor.YELLOW) {
            // Activate a servo or motor
            server.setPosition(1.0)
            }else {
            servo.setPosition(0.0);
            }
             */

        }

        //Stop streaming when OpMode is no longer active
        webcam.stopStreaming();
    }
}