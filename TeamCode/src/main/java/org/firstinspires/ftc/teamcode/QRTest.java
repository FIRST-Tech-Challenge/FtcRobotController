package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.ConeDnnProcessor;
import org.firstinspires.ftc.teamcode.hardware.ConeQRProcessor;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "QR Code Test")
public class QRTest  extends LinearOpMode  {

    OpenCvWebcam webCam;

    ConeQRProcessor coneImgPipeline;

    Hardware2022 hdw;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        hdw = new Hardware2022(hardwareMap,telemetry ); //init hardware

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webCamName  = hardwareMap.get(WebcamName.class, "Webcam 1");
        webCam = OpenCvCameraFactory.getInstance().createWebcam( webCamName    );

        coneImgPipeline = new ConeQRProcessor();
        webCam.setPipeline(coneImgPipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(1920,1080, OpenCvCameraRotation.SIDEWAYS_LEFT);
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
        telemetry.addData("Message : ", coneImgPipeline.getDetectMsg());
        telemetry.addData("Sleeve: ", coneImgPipeline.getSleeveSide()
                + " decoded:" + coneImgPipeline.isDecoded());
        telemetry.update();

        while (opModeIsActive())
        {

            if ( coneImgPipeline.isDecoded()) {
                //Once decoded, turn web stream off
                webCam.stopStreaming();

                telemetry.addData("Detected: ", coneImgPipeline.getDetectMsg());
                telemetry.addData("Sleeve: ", coneImgPipeline.getSleeveSide()
                        + " decoded:" + coneImgPipeline.isDecoded());
                telemetry.addData("Stop WebCam Streaming ", coneImgPipeline.getDetectMsg());
                telemetry.update();

            } else {
                telemetry.addData("Not Detected: ", coneImgPipeline.getDetectMsg());
                telemetry.update();
            }

            // Don't burn CPU cycles busy-looping in this sample
            sleep(100);
        }

    }



}
