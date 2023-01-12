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

import static org.firstinspires.ftc.teamcode.PowerPlaySuperPipeline.DebugObjects.ConeBlue;
import static org.firstinspires.ftc.teamcode.PowerPlaySuperPipeline.DebugObjects.Pole;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    double[]  TurretPwr   = new double[LOGSIZE];  // turret motor power

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

        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at or its
         * webcam counterpart,first.
         */

    public void performEveryLoop() {
        robot.readBulkData();
        robot.turretPosRun();
        robot.liftPosRun();
    }

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
                        false, false, 154.0);
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
                pipelineLow = new PowerPlaySuperPipeline(true, false,
                        false, true, 160.0);
                webcamLow.setPipeline(pipelineLow);
                webcamLow.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                pipelineLow.debugType = ConeBlue;
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

        telemetry.addLine("Hardware initialized...");
        telemetry.update();

        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );
        sleep(300);

        // Perform setup needed to center turret
//        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        robot.turretPosInit(-32.5 );
        robot.liftPosInit( robot.LIFT_ANGLE_HIGH_BA );
        while( robot.turretMotorAuto == true || robot.liftMotorAuto == true) {
            performEveryLoop();
        }
        robot.grabberSetTilt( robot.GRABBER_TILT_BACK_H );

        telemetry.addLine("Turret positioned, ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // Don't burn an insane amount of CPU cycles in this sample because
            // we're not doing anything else
            sleep(20);

            // Execute the automatic turret movement code
            performEveryLoop();

            // Let us see if we can use the camera for distance.
            alignToPoleTurretPID();
            telemetry.addLine("Aligned... waiting for kick");
            for( int index=0; index<LOGSIZE; index++ ) {
                telemetry.addData(" ","%d = %.1f %.1f pix (%.2f)",
                        index, angleOffset[index], distOffset[index], TurretPwr[index] );
            }
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

    /*---------------------------------------------------------------------------------*/

    void alignToPole() {
        PowerPlaySuperPipeline.AnalyzedPole theLocalPole;
        final double TURN_SLOPE   = 0.0000;   // power-per-pixel (not power-per-degree!) .15 deg/pixel
        final double TURN_OFFSET  = 0.1;
        final double DRIVE_SLOPE  = 0.004187;
        final double DRIVE_OFFSET = 0.04522;
        double turretPower;
        double drivePower;

        theLocalPole = pipelineBack.getDetectedPole();
        while (opModeIsActive() && ((theLocalPole.alignedCount <= 2) || theLocalPole.properDistanceHighCount <= 3)) {
            performEveryLoop();
            if(theLocalPole.aligned) {
                turretPower = 0.0;
            } else {
                // Need to calculate the turn power based on pixel offset
                // Maximum number of pixels off would be half of 320, so 160.
                // The FOV is 48 degrees, so 0.15 degrees per pixel. This should
                // go 1.0 to 0.08 from 24 degrees to 0.
                double minPower = (theLocalPole.centralOffset > 0)? -TURN_OFFSET : TURN_OFFSET;
                turretPower = (-theLocalPole.centralOffset * TURN_SLOPE) + minPower;
                if( turretPower < -0.25 ) turretPower = -0.25;
                if( turretPower > +0.25 ) turretPower = +0.25;
            }
            robot.setTurretPower(turretPower);

            if(theLocalPole.properDistanceHigh) {
                drivePower = 0.0;
            } else {
                // Need to calculate the drive power based on pixel offset
                // Maximum number of pixels off would be in the order of 30ish.
                // This is a first guess that will have to be expiremented on.
                // Go 1.0 to 0.08 from 30 pixels to 2.
                drivePower = (theLocalPole.highDistanceOffset > 0 )?
                        (theLocalPole.highDistanceOffset * DRIVE_SLOPE + DRIVE_OFFSET) :
                        (theLocalPole.highDistanceOffset * DRIVE_SLOPE - DRIVE_OFFSET);
            }
            if(abs(drivePower) < 0.01 && abs(turretPower) < 0.01) {
                robot.stopMotion();
                robot.setTurretPower(0);
            } else {
                driveAndRotateTurretAngle(drivePower, turretPower, true);
            }

            // Shift all previous instrumentation readings down one entry
            for( int index=1; index<LOGSIZE; index++ ) {
                angleOffset[index-1] = angleOffset[index];
                distOffset[index-1]  = distOffset[index];
                TurretPwr[index-1]   = TurretPwr[index];
            }
            // Add the latest numbers to the end
            angleOffset[LOGSIZE-1] = theLocalPole.centralOffset;
            distOffset[LOGSIZE-1]  = theLocalPole.highDistanceOffset;
            TurretPwr[LOGSIZE-1]   = turretPower;
            telemetry.addData("POLE","angle=%c distance=%c",
                    ((theLocalPole.aligned)? 'Y':'n'),
                    ((theLocalPole.properDistanceHigh)? 'Y':'n') );
            for( int index=0; index<LOGSIZE; index++ ) {
                telemetry.addData(" ","%d = %.1f  %.1f pix (%.2f)",
                        index, angleOffset[index], distOffset[index], TurretPwr[index] );
            }
            telemetry.update();
            // sleep( 40 );
            theLocalPole = pipelineBack.getDetectedPole();
        }
        robot.stopMotion();
        robot.setTurretPower(0);
    } // alignToPole

    void alignToPoleTurretPID() {
        PowerPlaySuperPipeline.AnalyzedPole theLocalPole;
        final double TURN_SLOPE   = 0.0000;   // power-per-pixel (not power-per-degree!) .15 deg/pixel
        final double TURN_OFFSET  = 0.1;
        final double DRIVE_SLOPE  = 0.004187;
        final double DRIVE_OFFSET = 0.04522;
        double turretPower;
        double drivePower;
        // PID stuff
        // Possible values 0.002, 0.005, 0.00005
        double kp = 0.0010;
        double ki = 0.0025;
        double kd = 0.00005;
        double error;
        double errorChange;
        double integralSum = 0.0;
        double derivative;
        double lastError = 0.0;
        boolean aligning = true;
        double a = 0.707;
        double currentFilterEstimate;
        double previousFilterEstimate = 0.0;
        // This value should be related to ki*integralSum where that value does not exceed
        // something like 25% max power (so if our max power is 0.20 the limit for ki*integralSum
        // would be 0.05 and 0.05 / ki = maxIntegralSum. Start high and bring it down once ki solved
        double maxIntegralSum = 50.0;
        ElapsedTime timer = new ElapsedTime();

        theLocalPole = pipelineBack.getDetectedPole();
        while (opModeIsActive() && ((theLocalPole.alignedCount <= 5) || theLocalPole.properDistanceHighCount <= 3)) {
            performEveryLoop();
            if(theLocalPole.aligned) {
                aligning = false;
                turretPower = 0.0;
            } else {
                if(!aligning) {
                    aligning = true;
                    lastError = 0.0;
                    integralSum = 0.0;
                }
                // The sign is backwards because centralOffset is negative of the power we need.
                error = 0.0 - theLocalPole.centralOffset;
                errorChange = error - lastError;
                currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
                previousFilterEstimate = currentFilterEstimate;
                derivative = currentFilterEstimate / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                if(integralSum > maxIntegralSum) integralSum = maxIntegralSum;
                if(integralSum < -maxIntegralSum) integralSum = -maxIntegralSum;
                turretPower = (kp * error) + (ki * integralSum) + (kd * derivative);
                lastError = error;
                timer.reset();
                telemetry.addData("turretPower: ", turretPower);
                telemetry.addData("PID", "error: %.2f, errorPwr: %.2f", error, kp*error);
                telemetry.addData("PID", "integralSum: %.2f, integralSumPwr: %.2f", integralSum, ki*integralSum);
                telemetry.addData("PID", "derivative: %.2f, derivativePwr: %.2f", derivative, ki*derivative);
                telemetry.update();
/*
                // Need to calculate the turn power based on pixel offset
                // Maximum number of pixels off would be half of 320, so 160.
                // The FOV is 48 degrees, so 0.15 degrees per pixel. This should
                // go 1.0 to 0.08 from 24 degrees to 0.
                double minPower = (theLocalPole.centralOffset > 0)? -TURN_OFFSET : TURN_OFFSET;
                turretPower = (-theLocalPole.centralOffset * TURN_SLOPE) + minPower;
                if( turretPower < -0.25 ) turretPower = -0.25;
                if( turretPower > +0.25 ) turretPower = +0.25;
 */
            }
            if(theLocalPole.properDistanceHigh) {
                drivePower = 0.0;
            } else {
                // Need to calculate the drive power based on pixel offset
                // Maximum number of pixels off would be in the order of 30ish.
                // This is a first guess that will have to be expiremented on.
                // Go 1.0 to 0.08 from 30 pixels to 2.
                drivePower = (theLocalPole.highDistanceOffset > 0 )?
                        (theLocalPole.highDistanceOffset * DRIVE_SLOPE + DRIVE_OFFSET) :
                        (theLocalPole.highDistanceOffset * DRIVE_SLOPE - DRIVE_OFFSET);
            }
            if(abs(drivePower) < 0.01 && abs(turretPower) < 0.01) {
                robot.stopMotion();
                robot.setTurretPower(0);
            } else {
                driveAndRotateTurretAngle(drivePower, turretPower, false);
            }

            theLocalPole = pipelineBack.getDetectedPole();
        }
        robot.stopMotion();
        robot.setTurretPower(0.0);
    } // alignToPoleTurretPID

    /**
     * Ensure angle is in the range of -180 to +180 deg (-PI to +PI)
     * This function won't have to be copied, it is part of auto base.
     * @param angleDegrees
     * @return
     */
    public double AngleWrapDegrees( double angleDegrees ){
        while( angleDegrees < -180 ) {
            angleDegrees += 360.0;
        }
        while( angleDegrees > 180 ){
            angleDegrees -= 360.0;
        }
        return angleDegrees;
    } // AngleWrapDegrees

    /*---------------------------------------------------------------------------------*/
    /*  AUTO: Drive at specified angle and power while turning at specified power.     */
    /*---------------------------------------------------------------------------------*/
    void driveAndRotateTurretAngle(double drivePower, double turretPower, boolean turretFacingFront) {
        double frontRight, frontLeft, rearRight, rearLeft, maxPower, xTranslation, yTranslation;
        double turretAngle = robot.turretAngle;

        // Correct the angle for the turret being in the back.
        if(!turretFacingFront) {
            turretAngle = AngleWrapDegrees(turretAngle + 180.0);
        }

        yTranslation = drivePower * Math.cos(toRadians(turretAngle));
        xTranslation = drivePower * Math.sin(toRadians(turretAngle));

        frontLeft  = yTranslation + xTranslation;
        frontRight = yTranslation - xTranslation;
        rearLeft   = yTranslation - xTranslation;
        rearRight  = yTranslation + xTranslation;

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
        robot.setTurretPower(turretPower);
    } // driveAndRotateTurretAngle
}