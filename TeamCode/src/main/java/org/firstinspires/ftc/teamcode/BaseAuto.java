package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.hardware.ConeImageProcessor;
import org.firstinspires.ftc.teamcode.hardware.ConeQRProcessor;
import org.firstinspires.ftc.teamcode.hardware.SleeveSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//import com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware2022;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;

//435 max ticks per second is 383.6

//@Autonomous(name = "BaseAuto")
public abstract class BaseAuto extends LinearOpMode {

    PtzControl myPtzControl;

    int curZoom = 200;

    OpenCvWebcam webCam;
    ConeQRProcessor coneImgPipeline;

    Hardware2022 hdw;
    MecanumWheels robot;
    int steps = 0;
    private ElapsedTime runtime = new ElapsedTime();

    SleeveSide currentSide = SleeveSide.Unkown;


    @Override
    public void runOpMode() {
        hdw = new Hardware2022(hardwareMap,telemetry ); //init hardware
        hdw.createHardware();

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webCamName  = hardwareMap.get(WebcamName.class, "Webcam 1");
        webCam = OpenCvCameraFactory.getInstance().createWebcam( webCamName    );

        coneImgPipeline = new ConeQRProcessor();
        webCam.setPipeline(coneImgPipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        //This is here so it starts recog after start
        waitForStart();

        //hdw.moveYAxis(6, 0.3);
        //sleep(100);

        hdw.moveYAxis(10, 1);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                myPtzControl = webCam.getPtzControl();
                myPtzControl.setZoom(curZoom);
                Log.d("9010", "min zoom" + myPtzControl.getMinZoom());
                Log.d("9010", "max zoom" + myPtzControl.getMaxZoom());


                webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        long startMills  = System.currentTimeMillis();

        //Max try 10 seconds.
        while ( !coneImgPipeline.isDecoded() & (System.currentTimeMillis() - startMills ) <15000 ) {
            telemetry.addData("Message : ", coneImgPipeline.getDetectMsg());
            telemetry.addData("Is decoded : ", coneImgPipeline.isDecoded());
            telemetry.addData("Sleeve: ", currentSide);
            telemetry.update();
            sleep(100);
        }
        currentSide = coneImgPipeline.getSleeveSide();
        webCam.closeCameraDevice();

        telemetry.addData("Message : ", coneImgPipeline.getDetectMsg());
        telemetry.addData("Sleeve: ", currentSide);
        telemetry.update();
        sleep(1000);


        scoreMidPole();

        switch ( currentSide) {
            case Sleev1: {
                parkZone1();
                break;
            }

            case Sleev2: {
                parkZone2();
                break;
            }

            case Sleev3: {
                parkZone3();
                break;
            }
            case Unkown: {
                parkTerminal();
                telemetry.addLine("Unknown, Move to Terminal");
                telemetry.update();
            }
        }

        while (opModeIsActive())  {
            idle();
        }

    }

    void parkZone1( ) {
        telemetry.addData("Park zone 1 ", this.currentSide);
        telemetry.update();
        //Move Left
        hdw.moveYAxis(17.0, -0.3);
        hdw.moveXAxis( -24.0, 0.3);

    }

    void parkZone2( ) {
        telemetry.addData("Park zone 2 ", this.currentSide);
        telemetry.update();
        //Move forward
        hdw.moveYAxis( 17.0, 0.3);

    }

    void parkZone3( ) {
        telemetry.addData("Park zone 3 ", this.currentSide);
        telemetry.update();
        //Move right
        hdw.moveYAxis(17, 0.3);
        hdw.moveXAxis( 24.0, 0.3);

    }


    abstract void parkTerminal();
    abstract void scoreMidPole();

}
