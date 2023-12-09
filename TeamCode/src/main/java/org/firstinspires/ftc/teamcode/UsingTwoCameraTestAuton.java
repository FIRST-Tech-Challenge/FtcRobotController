package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2Impl;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Autonomous
public class UsingTwoCameraTestAuton extends LinearOpMode {

        OpenCvCamera spikeCam;
        OpenCvCamera webcam;

        @Override
        public void runOpMode()
        {
            /**
             * NOTE: Many comments have been omitted from this sample for the
             * sake of conciseness. If you're just starting out with EasyOpenCV,
             * you should take a look at {@link InternalCamera1Example} or its
             * webcam counterpart, {@link WebcamExample} first.
             */

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            /**
             * This is the only thing you need to do differently when using multiple cameras.
             * Instead of obtaining the camera monitor view and directly passing that to the
             * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
             * on that view in order to split that view into multiple equal-sized child views,
             * and then pass those child views to the constructor.
             */
   /*         int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            cameraMonitorViewId, //The container we're splitting
                            2, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally

            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
*/
            WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
            WebcamName webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");



            AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();

            VisionPortal visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam2)
                    .addProcessor(aprilTag)
                    .build();
            spikeCam = OpenCvCameraFactory.getInstance().createWebcam(webcam1);

            PropLocationCam spikeLoc = new PropLocationCam();
            spikeLoc.initialize(this, PropLocationCam.TargetColor.RED,spikeCam);

            CyDogsAprilTags cyTags = new CyDogsAprilTags(this);
            cyTags.initAprilTag(aprilTag, visionPortal);
            CyDogsSparky mySparky = new CyDogsSparky(this);

            waitForStart();


            if (opModeIsActive()) {


                sleep(2000);
                AprilTagDetection foundTag = cyTags.GetAprilTag(1);
                PropLocationCam.location myLoc = spikeLoc.getSpikeLocation();
                telemetry.addData("Spike Location: ", myLoc.toString());
                telemetry.addData("Tag found?", foundTag.id );
                telemetry.addData("X distance", foundTag.ftcPose.x );
                // telemetry.addData("Internal cam FPS", phoneCam.getFps());
              //  telemetry.addData("Webcam FPS", webcam.getFps());
                telemetry.update();

                double inchesXMovement = foundTag.ftcPose.x;
                mySparky.StrafeRight((int)(inchesXMovement*22),.5,500);

                sleep(100);
            }
        }

    }





