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

import static org.firstinspires.ftc.teamcode.HardwareSlimbot.UltrasonicsInstances.SONIC_RANGE_FRONT;
import static org.firstinspires.ftc.teamcode.HardwareSlimbot.UltrasonicsModes.SONIC_FIRST_PING;
import static org.firstinspires.ftc.teamcode.HardwareSlimbot.UltrasonicsModes.SONIC_MOST_RECENT;
import static org.firstinspires.ftc.teamcode.PowerPlaySuperPipeline.DebugObjects.ConeBlue;
import static org.firstinspires.ftc.teamcode.PowerPlaySuperPipeline.DebugObjects.ConeRed;
import static org.firstinspires.ftc.teamcode.PowerPlaySuperPipeline.DebugObjects.Pole;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

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
    PowerPlaySuperPipeline pipelineBack;
    OpenCvCamera webcamLow = null;
    OpenCvCamera webcamBack = null;
    /* Declare OpMode members. */
    HardwareSlimbot robot = new HardwareSlimbot();
    boolean aligning = false;
    boolean ranging = false;
    boolean turretFacingFront = false;
    boolean lowCameraInitialized = false;
    boolean backCameraInitialized = false;
    boolean testBlue = true;
    final int LOGSIZE = 10;
    double[]  angleOffset = new double[LOGSIZE];  // pixel offset error (left/right)
    double[]  distOffset  = new double[LOGSIZE];  // pixel offset error (too narrow/wide)
    double[]  TurretPwr   = new double[LOGSIZE];  // turret motor power

    public void performEveryLoop() {
        robot.readBulkData();
        robot.turretPosRun();
        robot.liftPosRun();
    }
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally
        webcamBack = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Back"), viewportContainerIds[0]);
        webcamBack.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineBack = new PowerPlaySuperPipeline(false, true,
                        false, false, 144.0);
                webcamBack.setPipeline(pipelineBack);
                webcamBack.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                pipelineLow.debugType = Pole;
                backCameraInitialized = true;
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });
        webcamBack.showFpsMeterOnViewport(false);

        webcamLow = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Low"), viewportContainerIds[1]);
        webcamLow.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineLow = new PowerPlaySuperPipeline(false, false,
                        !testBlue, testBlue, 160.0);
                webcamLow.setPipeline(pipelineLow);
                webcamLow.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                pipelineLow.debugType = testBlue ? ConeBlue : ConeRed;
                lowCameraInitialized = true;
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });
        webcamLow.showFpsMeterOnViewport(false);

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        while(!(lowCameraInitialized && backCameraInitialized)) {
            sleep(100);
        }
        telemetry.addLine("Cameras initialized...");
        telemetry.update();

        /* Declare OpMode members. */
        robot.init(hardwareMap,false);

        telemetry.addLine("Robot initialized, ready to start.");
        telemetry.update();
        waitForStart();

        // Perform setup needed to center turret
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );

        while (opModeIsActive())
        {
            // Let us see if we can use the camera for distance.
            alignToConeStack(testBlue, 30);
            telemetry.addLine("Aligned... waiting for kick");
            telemetry.update();
            sleep( 2000 );
        }
        // Close out the vision
        if(webcamLow != null) {
            webcamLow.stopStreaming();
        }
        if(webcamBack != null) {
            webcamBack.stopStreaming();
        }
    }

    /*
     * @param blueCone - true = blue cone stack, false = red cone stack.
     * @param targetDistance - distance in cm from the cone stack to line up.
     */
    void alignToConeStack(boolean blueCone, int targetDistance) {
        PowerPlaySuperPipeline.AnalyzedCone theLocalCone;
        final double DRIVE_SLOPE  = 0.004187;
        final double DRIVE_OFFSET = 0.04522;
        final double TURN_SLOPE   = 0.004187;
        final double TURN_OFFSET  = 0.04522;
        double drivePower;
        double turnPower;
        int coneDistance;
        int distanceError;
        int properDistanceCount = 0;
        // This is in CM
        final int MAX_DISTANCE_ERROR = 2;

        theLocalCone = blueCone ? pipelineLow.getDetectedBlueCone() : pipelineLow.getDetectedRedCone();
        // This first reading just triggers the ultrasonic to send out its first ping, need to wait
        // 50 msec for it to get a result.
        robot.fastSonarRange(SONIC_RANGE_FRONT, SONIC_FIRST_PING);
        sleep(50);
        while (opModeIsActive() && ((theLocalCone.alignedCount <= 3) ||
                properDistanceCount <= 3)) {
            theLocalCone = blueCone ? pipelineLow.getDetectedBlueCone() : pipelineLow.getDetectedRedCone();
            coneDistance = robot.fastSonarRange(SONIC_RANGE_FRONT, SONIC_MOST_RECENT);
            distanceError = targetDistance - coneDistance;
            if(abs(distanceError) <= MAX_DISTANCE_ERROR) {
                properDistanceCount++;
            } else {
                properDistanceCount = 0;
            }
            drivePower = (distanceError > 0) ? (-distanceError * DRIVE_SLOPE - DRIVE_OFFSET) :
                    (-distanceError * DRIVE_SLOPE + DRIVE_OFFSET);
            turnPower = (theLocalCone.centralOffset > 0) ?
                    (theLocalCone.centralOffset * TURN_SLOPE + TURN_OFFSET) :
                    (theLocalCone.centralOffset * TURN_SLOPE - TURN_OFFSET);
            driveAndRotate(drivePower, turnPower);
            telemetry.addData("Cone Data", "drvPwr %.2f turnPwr %.2f", drivePower, turnPower);
            telemetry.addData("Cone Data", "coneDst %d dstErr %d offset %.2f", coneDistance,
                    distanceError, theLocalCone.centralOffset);
            telemetry.update();
        }
        robot.stopMotion();
    }

    void driveAndRotate(double drivePower, double turnPower) {
        double frontRight, frontLeft, rearRight, rearLeft, maxPower;

        frontLeft  = drivePower - turnPower;
        frontRight = drivePower + turnPower;
        rearLeft   = drivePower - turnPower;
        rearRight  = drivePower + turnPower;

        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(rearLeft),  Math.abs(rearRight)  ),
                Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            rearLeft   /= maxPower;
            rearRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }
        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
    } // driveAndRotate
}