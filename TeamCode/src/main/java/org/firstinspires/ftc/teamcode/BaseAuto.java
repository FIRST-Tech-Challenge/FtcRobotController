package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ConeImageProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

//import com.qualcomm.robotcore.hardware;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware2022;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;

//435 max ticks per second is 383.6


public abstract class BaseAuto extends LinearOpMode {

    OpenCvInternalCamera phoneCam;

    ConeImageProcessor ConeImgPipeline;

    Hardware2022 hdw;
    MecanumWheels robot;
    int steps = 0;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        hdw = new Hardware2022(hardwareMap,telemetry ); //init hardware
        hdw.createHardware();
        robot = new MecanumWheels();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        ConeImgPipeline = new ConeImageProcessor();
        phoneCam.setPipeline(ConeImgPipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
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

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", ConeImgPipeline.getMeanVal());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }


    }

    abstract void movePark();

}
