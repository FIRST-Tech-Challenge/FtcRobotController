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

            // Let us see if we can use the camera for distance.
            alignToPole();
            telemetry.addLine("Aligned... waiting for kick");
            telemetry.update();
        }
    }

    /*---------------------------------------------------------------------------------*/
    void alignToPole() {
        PowerPlaySuperPipeline.AnalyzedPole theLocalPole;
        double turnPower;
        double drivePower;
        double fr, fl, br, bl;
        theLocalPole = new PowerPlaySuperPipeline.AnalyzedPole(pipeline.getDetectedPole());
//      while (opModeIsActive() && (theLocalPole.alignedCount < 2) && theLocalPole.properDistanceHighCount < 2) {
        while (opModeIsActive() ) {
            if(theLocalPole.poleAligned) {
                turnPower = 0.0;
            } else {
                // Rotate right or left
                turnPower = theLocalPole.centralOffset > 0 ? 0.085 : -0.085;
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
            telemetry.addData("POLE","angle=%c offset=%.1f pix",
                    ((theLocalPole.poleAligned)? 'Y':'n'),
                    theLocalPole.centralOffset );
            telemetry.addData("POLE","distance=%c offset=%.1f width=%.1f pix",
                    ((theLocalPole.properDistanceHigh)? 'Y':'n'),
                    theLocalPole.highDistanceOffset,
                    theLocalPole.corners.size.height );
            telemetry.update();
            theLocalPole = new PowerPlaySuperPipeline.AnalyzedPole(pipeline.getDetectedPole());
        }
        robot.stopMotion();
    } // alignToPole
}