package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.ConeImageProcessor;
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

@Autonomous(name = "BaseAuto")
public  class BaseAuto extends LinearOpMode {

    OpenCvWebcam webcam;
    ConeImageProcessor ConeImgPipeline;

    Hardware2022 hdw;
    MecanumWheels robot;
    int steps = 0;
    private ElapsedTime runtime = new ElapsedTime();

    SleeveSide currentSide = SleeveSide.Unkown;


    @Override
    public void runOpMode() {
        hdw = new Hardware2022(hardwareMap,telemetry ); //init hardware
        hdw.createHardware();
        robot = new MecanumWheels();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        ConeImgPipeline = new ConeImageProcessor();
        webcam.setPipeline(ConeImgPipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("Carema Error , " ,  errorCode);
                telemetry.update();
                sleep(1000);
            }
        });

        while (!isStarted() ) {
            currentSide = ConeImgPipeline.getSleveSide();
            telemetry.addData("Realtime analysis", this.currentSide);
            telemetry.addData("Mean Value", ConeImgPipeline.getMeanVal());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        //phoneCam.closeCameraDevice();

        waitForStart();
        telemetry.addData("Post Start analysis", this.currentSide);
        telemetry.addData("Mean Value", ConeImgPipeline.getMeanVal());
        telemetry.update();

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
                parkZone2();

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
        hdw.moveXAxis(24.0, -0.1);
        hdw.moveYAxis( 36.0, 0.3);
    }

    void parkZone2( ) {
        telemetry.addData("Park zone 2 ", this.currentSide);
        telemetry.update();
        //Move forward
        hdw.moveYAxis( 36.0, 0.3);

    }

    void parkZone3( ) {
        telemetry.addData("Park zone 3 ", this.currentSide);
        telemetry.update();
        //Move right
        hdw.moveXAxis(24.0, 0.1);
        hdw.moveYAxis( 36.0, 0.3);

    }


    //abstract void movePark();

}
