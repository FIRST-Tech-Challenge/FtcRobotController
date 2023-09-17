package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFVisionPortal;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Config
@TeleOp(name="Optimize AprilTag Exposure", group = "Concept")

public class TestAprilTag extends LinearOpMode{
        public static double decimate = 5;
        private RFVisionPortal visionPortal = null;        // Used to manage the video source.
        private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
        private int     myExposure  ;
        private int     minExposure ;
        private int     maxExposure ;
        private int     myGain      ;
        private int     minGain ;
        private int     maxGain ;

        boolean thisExpUp = false;
        boolean thisExpDn = false;
        boolean thisGainUp = false;
        boolean thisGainDn = false;

        boolean lastExpUp = false;
        boolean lastExpDn = false;
        boolean lastGainUp = false;
        boolean lastGainDn = false;
        double getDecimate = 0;
         public static double currentSolver =1;
         double loopNum;
        @SuppressLint("DefaultLocale")
        @Override public void runOpMode()
        {
            BasicRobot robot = new BasicRobot(this, false);
            // Initialize the Apriltag Detection process
            initAprilTag();


            // Wait for the match to begin.
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            while (opModeIsActive())
            {
                telemetry.addLine("Find lowest Exposure that gives reliable detection.");
                telemetry.addLine("Use Left bump/trig to adjust Exposure.");
                telemetry.addLine("Use Right bump/trig to adjust Gain.\n");

                // Display how many Tags Detected
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                int numTags = currentDetections.size();

                if(gamepad1.a) {
                    if(visionPortal.getCameraState()== RFVisionPortal.CameraState.STREAMING) {
                        visionPortal.stopStreaming();
                    }
                    else {
                        visionPortal.resumeStreaming();
                    }
                };
//                if(gamepad1.dpad_up) {
                    if((int)currentSolver==0) {
                        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);
//                        currentSolver++;
                    }
                    else if((int)currentSolver==1) {
                        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE);
//                        currentSolver++;
                    }
                    else if((int)currentSolver==3) {
                        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE);
//                        currentSolver++;
                    }else if((int)currentSolver==4) {
                        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SQPNP);
//                        currentSolver++;
                    }else if((int)currentSolver==5) {
                        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
//                        currentSolver=0;
                    }else if((int)currentSolver==2) {
                        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE_SQUARE);
//                        currentSolver++;
                    }
//                }
                packet.put("currentSolver", currentSolver);
                packet.put("poseSolveTime", aprilTag.getPerTagAvgPoseSolveTime());
//                if(decimate!=getDecimate) {
                    aprilTag.setDecimation((float) decimate);
//                    getDecimate = decimate;
//                }
                packet.put("fps", visionPortal.getFps());

                telemetry.addData("# AprilTags Detected", currentDetections.size());

                // Step through the list of detections and display info for each one.
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
//                        packet.put("X", detection.ftcPose.x);
//                        packet.put("Y", detection.ftcPose.y);
//                        packet.put("Z", detection.ftcPose.z);
//                        packet.put("yaw", detection.ftcPose.yaw);
//                        packet.put("pitch", detection.ftcPose.pitch);
                        packet.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        packet.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        packet.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        packet.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing,
                                detection.ftcPose.elevation));
                        break;
                    }
                }
                robot.update();


                lastExpUp = thisExpUp;
                lastExpDn = thisExpDn;
                lastGainUp = thisGainUp;
                lastGainDn = thisGainDn;
                loopNum++;
                packet.put("loopTime", loopNum/BasicRobot.time);
//                sleep(30);
            }
        }

        /**
         * Initialize the AprilTag processor.
         */
        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                    .build();
            aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE);

            // Create the WEBCAM vision portal by using a builder.
            visionPortal = new RFVisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .enableLiveView(false)
                    .setStreamFormat(RFVisionPortal.StreamFormat.MJPEG)
                    .setCameraResolution(new Size(640,480))
                    .build();
        }

        /*
            Manually set the camera gain and exposure.
            Can only be called AFTER calling initAprilTag();
            Returns true if controls are set.
         */
        private boolean setManualExposure(int exposureMS, int gain) {
            // Ensure Vision Portal has been setup.
            if (visionPortal == null) {
                return false;
            }

            // Wait for the camera to be open
            if (visionPortal.getCameraState() != RFVisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != RFVisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);

                // Set Gain.
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
                return (true);
            } else {
                return (false);
            }
        }

        /*
            Read this camera's minimum and maximum Exposure and Gain settings.
            Can only be called AFTER calling initAprilTag();
         */
    }


