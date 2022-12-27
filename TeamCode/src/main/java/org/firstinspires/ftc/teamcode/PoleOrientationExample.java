/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple poles, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@TeleOp(name="Pole-Test", group="Skunkworks")
public class PoleOrientationExample extends LinearOpMode
{
    final int LOGSIZE = 10;
    double[]  angleOffset = new double[LOGSIZE];  // pixel offset error (left/right)
    double[]  distOffset  = new double[LOGSIZE];  // pixel offset error (too narrow/wide)

    // Vision stuff
    PowerPlaySuperPipeline pipelineLow;
    PowerPlaySuperPipeline pipelineFront;
    PowerPlaySuperPipeline pipelineBack;
    OpenCvCamera webcamLow = null;
    OpenCvCamera webcamFront = null;
    OpenCvCamera webcamBack = null;
    /* Declare OpMode members. */
//    HardwareSlimbot robot = new HardwareSlimbot();
    boolean aligning = false;
    boolean ranging = false;
    PowerPlaySuperPipeline.AnalyzedPole thePole;

        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at or its
         * webcam counterpart,first.
         */


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Robot initializing, wait for completion.");
        telemetry.update();

        int cameraCount = 2;
        // Create camera instances
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        cameraCount, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally
        if(cameraCount >= 1) {
            webcamLow = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                    "Webcam Low"), viewportContainerIds[0]);
            pipelineLow = new PowerPlaySuperPipeline(true, false,
                    false, true, 160.0, true,
                    false);
            webcamLow.openCameraDevice();
            webcamLow.setPipeline(pipelineLow);
            webcamLow.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            webcamLow.showFpsMeterOnViewport(false);
        }
        if(cameraCount >= 2) {
            webcamFront = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                    "Webcam Front"), viewportContainerIds[1]);
            pipelineFront = new PowerPlaySuperPipeline(false, true,
                    false, false, 160.0, true,
                    false);
            webcamFront.openCameraDevice();
            webcamFront.setPipeline(pipelineFront);
            webcamFront.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            webcamFront.showFpsMeterOnViewport(false);
        }
        if(cameraCount >= 3) {
            webcamBack = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                    "Webcam Back"), viewportContainerIds[2]);
            pipelineBack = new PowerPlaySuperPipeline(false, true,
                    false, false, 160.0, true,
                    false);
            webcamBack.openCameraDevice();
            webcamBack.setPipeline(pipelineBack);
            webcamBack.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            webcamBack.showFpsMeterOnViewport(false);
        }

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        /* Declare OpMode members. */
//        robot.init(hardwareMap,false);

        telemetry.addLine("Robot initialized, ready to start.");
        telemetry.update();
        waitForStart();

        // Perform setup needed to center turret
//        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );

        while (opModeIsActive())
        {
            // Don't burn an insane amount of CPU cycles in this sample because
            // we're not doing anything else
            sleep(20);

            // Execute the automatic turret movement code   
//            robot.readBulkData();
//            robot.turretPosRun();

            // Let us see if we can use the camera for distance.
//            alignToPole();
            telemetry.addLine("Aligned... waiting for kick");
            for( int index=0; index<LOGSIZE; index++ ) {
                telemetry.addData(" ","%d = %.1f %.1f pix",
                        index, angleOffset[index], distOffset[index] );
            }
            telemetry.update();
            sleep( 2000 );
        }

        // Close out the vision
        if(webcamLow != null) {
            webcamLow.stopStreaming();
        }
        if(webcamFront != null) {
            webcamFront.stopStreaming();
        }
        if(webcamBack != null) {
            webcamBack.stopStreaming();
        }
//        webcamLow.closeCameraDevice();
//        webcamFront.closeCameraDevice();
//        webcamBack.closeCameraDevice();
    }

    /*---------------------------------------------------------------------------------*/
    /*
    void alignToPole() {
        PowerPlaySuperPipeline.AnalyzedPole theLocalPole;
        double turnPower;
        double drivePower;
        double fr, fl, br, bl;
        theLocalPole = new PowerPlaySuperPipeline.AnalyzedPole(pipelineFront.getDetectedPole());
        while (opModeIsActive() && ((theLocalPole.alignedCount <= 3) || theLocalPole.properDistanceHighCount <= 3)) {
//      while (opModeIsActive() ) {
            if(theLocalPole.poleAligned) {
                turnPower = 0.0;
            } else {
                // Rotate right or left
                turnPower = theLocalPole.centralOffset > 0 ? 0.08 : -0.08;
            }
            if(theLocalPole.properDistanceHigh) {
                drivePower = 0.0;
            } else {
                drivePower = theLocalPole.highDistanceOffset > 0 ? 0.10 : -0.10;
            }
            if(abs(drivePower) < 0.01 && abs(turnPower) < 0.01) {
                robot.stopMotion();
            } else {
                fl = drivePower - turnPower;
                fr = drivePower + turnPower;
                bl = drivePower - turnPower;
                br = drivePower + turnPower;
                robot.driveTrainMotors(fl, fr, bl, br);
            }

            // Shift all previous instrumentation readings down one entry
            for( int index=1; index<LOGSIZE; index++ ) {
                angleOffset[index-1] = angleOffset[index];
                distOffset[index-1]  = distOffset[index];
            }
            // Add the latest numbers to the end
            angleOffset[LOGSIZE-1] = theLocalPole.centralOffset;
            distOffset[LOGSIZE-1]  = theLocalPole.highDistanceOffset;
            telemetry.addData("POLE","angle=%c distance=%c",
                    ((theLocalPole.poleAligned)? 'Y':'n'),
                    ((theLocalPole.properDistanceHigh)? 'Y':'n') );
            for( int index=0; index<LOGSIZE; index++ ) {
                telemetry.addData(" ","%d = %.1f  %.1f pix",
                        index, angleOffset[index], distOffset[index] );
            }
            telemetry.update();
            // sleep( 40 );
            theLocalPole = new PowerPlaySuperPipeline.AnalyzedPole(pipelineFront.getDetectedPole());
        }
        robot.stopMotion();
    } // alignToPole
     */
}