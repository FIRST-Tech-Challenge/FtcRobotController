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
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple cones, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@TeleOp(name="Cone-Test", group="Skunkworks")
public class ConeOrientationExample extends LinearOpMode
{
    // Vision stuff
    PowerPlaySuperPipeline pipelineLow;
    PowerPlaySuperPipeline pipelineFront;
    PowerPlaySuperPipeline pipelineBack;
    OpenCvCamera webcamLow;
    OpenCvCamera webcamFront;
    OpenCvCamera webcamBack;
    PowerPlaySuperPipeline pipeline;
    /* Declare OpMode members. */
    HardwareSlimbot robot = new HardwareSlimbot();
    boolean aligning = false;
    boolean ranging = false;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera2Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        3, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally
        webcamLow = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Low"), viewportContainerIds[0]);
        webcamLow.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineLow = new PowerPlaySuperPipeline(true, false,
                        false, false, 160.0, true, false);
                webcamLow.setPipeline(pipelineLow);
                webcamLow.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });

        webcamFront = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Front"), viewportContainerIds[1]);
        webcamFront.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineFront = new PowerPlaySuperPipeline(false, true,
                        false, false, 160.0, true, false);
                webcamFront.setPipeline(pipelineFront);
                webcamFront.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

           @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });

        webcamBack = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Back"), viewportContainerIds[2]);
        webcamBack.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineBack = new PowerPlaySuperPipeline(false, true,
                        false, false, 160.0, true, false);
                webcamBack.setPipeline(pipelineBack);
                webcamBack.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        /* Declare OpMode members. */
        robot.init(hardwareMap,false);

        waitForStart();

        // Perform setup needed to center turret
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );

        PowerPlaySuperPipeline.AnalyzedCone cone = new PowerPlaySuperPipeline.AnalyzedCone();
        cone.corners = new RotatedRect(new double[]{0.0, 0.0, 0.0, 0.0});
        cone.centralOffset = 0;
        cone.coneAligned = false;
        PowerPlaySuperPipeline.AnalyzedCone lastCone = new PowerPlaySuperPipeline.AnalyzedCone();
        lastCone.corners = new RotatedRect(new double[]{0.0, 0.0, 0.0, 0.0});
        lastCone.centralOffset = 0;
        lastCone.coneAligned = false;
        while (opModeIsActive())
        {
            // Don't burn an insane amount of CPU cycles in this sample because
            // we're not doing anything else
            sleep(20);

            // Execute the automatic turret movement code
            robot.readBulkData();
            robot.turretPosRun();

            // Figure out which poles the pipeline detected, and print them to telemetry
            cone = pipeline.getDetectedRedCone();
            if(cone != null) {
                telemetry.addLine(String.format("Cone: Center=%s, Central Offset=%f, Centered:%s", cone.corners.center.toString(), cone.centralOffset, cone.coneAligned));
                telemetry.addLine(String.format("Cone Width=%f Cone Height=%f", cone.corners.size.width, cone.corners.size.height));
                // Ensure we're ALIGNED to pole before we attempt to use Ultrasonic RANGING
                if (!cone.coneAligned) {
                    aligning = true;
//              rotateToCenterCone(cones.get(0));
                }
                // We've achieved ALIGNMENT, so halt the left/right rotation
                else if (aligning) {
                    robot.stopMotion();
                    aligning = false;
                    ranging = true;
                }
                // If aligned, adjust the distance to the pole
                if (cone.coneAligned && ranging) {
//              distanceToCone();
                }
            }
            telemetry.addData("Sonar Range (Front)", "%.1f", robot.updateSonarRangeF() );
            telemetry.update();
        }
    }

    void distanceToCone() {
        // Value in inches?
        double desiredDistance = 28.0;
        double distanceTolerance = 1.0;
        double range = robot.updateSonarRangeF();
        double rangeErr = range - desiredDistance;
        if( abs(rangeErr) > distanceTolerance ) {
            // Drive towards/away from the pole
            robot.driveTrainFwdRev( (rangeErr>0.0)? +0.10 : -0.10 );
        } else {
            robot.stopMotion();
            ranging = false;
        }
    }

    void rotateToCenterCone(PowerPlaySuperPipeline.AnalyzedCone theCone)
    {
        robot.driveTrainTurn( (theCone.centralOffset>0)? +0.10 : -0.10 );
    }
}