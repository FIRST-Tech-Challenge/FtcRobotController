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
import java.util.List;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple poles, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@TeleOp(name="Pole-Test", group="Skunkworks")
public class PoleOrientationExample extends LinearOpMode
{
    OpenCvCamera webcam;
    /* Declare OpMode members. */
    HardwareSlimbot robot = new HardwareSlimbot();
    boolean aligning = false;
    boolean ranging = false;
    PowerPlaySuperPipeline.AnalyzedPole thePole;

        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at or its
         * webcam counterpart,first.
         */
        PowerPlaySuperPipeline pipeline;


    @Override
    public void runOpMode() throws InterruptedException {
        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                pipeline = new PowerPlaySuperPipeline(true, false, false, 160.0);
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        /* Declare OpMode members. */
        robot.init(hardwareMap,false);

        waitForStart();

        // Perform setup needed to center turret
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );

        while (opModeIsActive())
        {
            // Don't burn an insane amount of CPU cycles in this sample because
            // we're not doing anything else
            sleep(20);

            // Execute the automatic turret movement code   
            robot.readBulkData();
            robot.turretPosRun();

            rotateToCenterPole();
            telemetry.addLine("Just a stupid line.");
            telemetry.update();
        }
    }

    void distanceToPole() {
        // Value in inches?
        double desiredDistance = 28.0;
        double distanceTolerance = 1.0;
        double range = robot.updateSonarRangeF();
        double rangeErr = range - desiredDistance;
        if( abs(rangeErr) > distanceTolerance ) {
            // Drive towards/away from the pole
//            robot.driveTrainFwdRev( (rangeErr>0.0)? +0.10 : -0.10 );
        } else {
            robot.stopMotion();
            ranging = false;
        }
    }

    /*---------------------------------------------------------------------------------*/
    void rotateToCenterPole() {
        int alignedCount = 0;
        synchronized(pipeline.lockPole) {
            List<PowerPlaySuperPipeline.AnalyzedPole> localPoles = pipeline.getDetectedPoles();
            if (!localPoles.isEmpty()) {
                thePole = new PowerPlaySuperPipeline.AnalyzedPole(localPoles.get(0));
                alignedCount = thePole.alignedCount;
            }
        }
        int loops = 0;
        while (opModeIsActive() && (alignedCount < 10)) {
            loops++;
            telemetry.addData("RotateToPole", "Loop count: %d", loops);
            telemetry.addData("RotateToPole", "AlignmentCount: %d", alignedCount);
            if(thePole != null) {
                if(thePole.poleAligned) {
                    telemetry.addData("RotateToPole", "Robot Aligned!");
                    robot.stopMotion();
                } else {
                    telemetry.addData("RotateToPole", "Robot Not Aligned!");
                    robot.driveTrainTurn((thePole.centralOffset > 0) ? +0.07 : -0.07);
                }
            }
            synchronized(pipeline.lockPole) {
                List<PowerPlaySuperPipeline.AnalyzedPole> localPoles = pipeline.getDetectedPoles();
                if (!localPoles.isEmpty()) {
                    telemetry.addData("RotateToPole", "Getting new pole.");
                    thePole = new PowerPlaySuperPipeline.AnalyzedPole(localPoles.get(0));
                    alignedCount = thePole.alignedCount;
                } else {
                    telemetry.addData("RotateToPole", "Returned Pole list is empty.");
                }
            }
            telemetry.update();
        }
        robot.stopMotion();
        // TODO: can we use this aligned position along the tape to update our known
        // odometry world position? (offsetting any drift-error that has accumulated?
    } // rotateToCenterPole
}