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
@TeleOp(name="Turret-Test", group="Skunkworks")
//@Disabled
public class TurretPIDTester extends AutonomousBase
{
    boolean alignToFront = true;
    OpenCvCamera webcamBack;
    OpenCvCamera webcamFront;
    OpenCvCamera webcamLow;
    PowerPlaySuperPipeline pipelineLow;
    PowerPlaySuperPipeline pipelineFront;
    ElapsedTime autonomousTimer = new ElapsedTime();
    final int LOGSIZE = 12;
    double[]  errorHistory = new double[LOGSIZE];
    double[]  kpMinHistory = new double[LOGSIZE];
    double[]  kpHistory    = new double[LOGSIZE];
    double[]  kiHistory    = new double[LOGSIZE];
    double[]  kdHistory    = new double[LOGSIZE];
    double[]  kTHistory    = new double[LOGSIZE];

    /* Declare OpMode members. */
    boolean aligning = false;
    boolean ranging = false;
    boolean turretFacingFront = false;
    boolean lowCameraInitialized = false;
    boolean frontCameraInitialized = false;
    boolean backCameraInitialized = false;
    double maxPower = 0.0;

    double currentPositionX                     = 0.0;   // Keeps track of our Autonomous alignToPole() target position
    double currentPositionY                     = 0.0;
    double currentPositionAngle                 = 0.0;

    double targetPositionX                      = 0.0;   // Keeps track of our Autonomous alignToPole() target position
    double targetPositionY                      = 0.0;
    double targetPositionAngle                  = 0.0;

    double targetDistanceX                      = 0.0;   // How far do we need to move robot base to alignToPole()?
    double targetDistanceY                      = 0.0;

    double targetAngle                          = 0.0;   // Keeps track of our Autonomous alignToPole() target turret angle

    /**
     * NOTE: Many comments have been omitted from this sample for the
     * sake of conciseness. If you're just starting out with EasyOpenCv,
     * you should take a look at or its
     * webcam counterpart,first.
     */


    ElapsedTime intakeTimer = new ElapsedTime();
    /*--------------------------------------------------------------------------------------------*/
    private void scoreCone() {

        // Start ejecting the cone
        intakeTimer.reset();
        // Wait for sensor to indicate it's clear (or we timeout)
        while( opModeIsActive() ) {
            performEveryLoop();
            // Ensure we eject for at least 250 msec before using sensor (in case sensor fails)
            boolean bottomSensorClear = robot.bottomConeState && (intakeTimer.milliseconds() > 250);
            // Also have a max timeout in case sensor fails
            boolean maxEjectTimeReached = (intakeTimer.milliseconds() >= 400);
            // Is cycle complete?
            if( bottomSensorClear || maxEjectTimeReached) break;
        }
        // Stop the ejector
        robot.grabberSetTilt( robot.GRABBER_TILT_GRAB3 );

    } // scoreCone

    public void performAutoNewLift() {
        // Now reverse the lift to raise off the cone stack
        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH );
        while( opModeIsActive() && (robot.liftAngle >= robot.LIFT_ANGLE_MOTORS) ) {
            performEveryLoop();
        }
        robot.turretPIDPosInit( 118.0 );

        // Perform setup to center turret and raise lift to scoring position
        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );
        while( opModeIsActive() && ( robot.turretMotorPIDAuto == true || robot.liftMotorPIDAuto == true )) {
            performEveryLoop();
        }

        scoreCone();

        robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPIDPosInit( robot.LIFT_ANGLE_5STACK );
        while( opModeIsActive() && ((robot.turretMotorPIDAuto == true) || (robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }
        robot.liftPIDPosInit( robot.LIFT_ANGLE_COLLECT );
        while( opModeIsActive() && ((robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }
    }

    public void performTeleNewLift(double collectAngle, double scoreAngle) {
        // Now reverse the lift to raise off the cone stack
        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH );
        while( opModeIsActive() && (robot.liftAngle >= robot.LIFT_ANGLE_MOTORS) ) {
            performEveryLoop();
        }
        robot.turretPIDPosInit( scoreAngle );

        // Perform setup to center turret and raise lift to scoring position
        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );
        while( opModeIsActive() && ( robot.turretMotorPIDAuto == true || robot.liftMotorPIDAuto == true )) {
            performEveryLoop();
        }

        scoreCone();

        robot.turretPIDPosInit( collectAngle );
        robot.liftPIDPosInit( robot.LIFT_ANGLE_COLLECT );
        while( opModeIsActive() && ((robot.turretMotorPIDAuto == true) || (robot.liftMotorPIDAuto == true)) ) {
            performEveryLoop();
        }
    }

    void alignToPole(boolean turretFacingFront) {
        PowerPlaySuperPipeline alignmentPipeline;
        PowerPlaySuperPipeline.AnalyzedPole theLocalPole;
        final double DRIVE_SLOPE  = 0.004187;
        final double DRIVE_OFFSET = 0.04522;
        final int TURRET_CYCLES_AT_POS = 8;
        // minPower=0; kp = 0.0027
        PIDControllerTurret pidController = new PIDControllerTurret(0.00008,0.000, 0.00010, 0.085 );

        double turretPower;
        double turretPowerMax = 0.14;  // maximum we don't want the PID to exceed
        double drivePower;

        double startTime = autonomousTimer.milliseconds();
        double abortTime = startTime + 3000.0;  // abort after 3 seconds

        // If we add back front camera, use boolean to determine which pipeline to use.
        alignmentPipeline = turretFacingFront ? pipelineFront : pipelineBack;

        theLocalPole = alignmentPipeline.getDetectedPole();
		// Save the starting image we have to correct for
        alignmentPipeline.savePoleAutoImage();

        while (opModeIsActive() && ((theLocalPole.alignedCount <= TURRET_CYCLES_AT_POS) ||
                theLocalPole.properDistanceHighCount <= 3)) {
            performEveryLoop();
            turretPower = pidController.update(0.0, theLocalPole.centralOffset);
            // Ensure we never exceed a safe power
            if( turretPower > +turretPowerMax ) turretPower = +turretPowerMax;
            if( turretPower < -turretPowerMax ) turretPower = -turretPowerMax;

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

            telemetry.addData("alignToPole", "ang=%d (%.1f) dist=%d (%.1f)",
                    theLocalPole.alignedCount, theLocalPole.centralOffset,
                    theLocalPole.properDistanceHighCount, theLocalPole.highDistanceOffset );
            telemetry.update();

            if(abs(drivePower) < 0.01 && abs(turretPower) < 0.01) {
                robot.stopMotion();
                robot.turretMotorSetPower(0);
            } else {
                driveAndRotateTurretAngle(drivePower, turretPower, turretFacingFront);
            }

            // Do we need to abort due to timeout?
            if( autonomousTimer.milliseconds() >= abortTime )
                break;

            // update the image for the next loop
            theLocalPole = alignmentPipeline.getDetectedPole();
        }
        robot.stopMotion();
        robot.turretMotorSetPower(0.0);
        // Save the final image after our adjustments
        alignmentPipeline.savePoleAutoImage();
    } // alignToPole

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime;
        double newTime;
        double target = 20.0;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

        if(alignToFront) {
            webcamFront = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                    "Webcam Front"), viewportContainerIds[0]);
            webcamFront.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    pipelineFront = new PowerPlaySuperPipeline(false, true, false, false, 160.0);
                    webcamFront.setPipeline(pipelineFront);
                    webcamFront.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    frontCameraInitialized = true;
                }

                @Override
                public void onError(int errorCode) {
                    // This will be called if the camera could not be opened
                }
            });
            webcamFront.showFpsMeterOnViewport(false);
        } else {
            webcamBack = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                    "Webcam Back"), viewportContainerIds[0]);
            webcamBack.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    pipelineBack = new PowerPlaySuperPipeline(false, true, false, false, 160.0);
                    webcamBack.setPipeline(pipelineBack);
                    webcamBack.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    backCameraInitialized = true;
                }

                @Override
                public void onError(int errorCode) {
                    // This will be called if the camera could not be opened
                }
            });
            webcamBack.showFpsMeterOnViewport(false);
        }

        webcamLow = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam Low"), viewportContainerIds[1]);
        webcamLow.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                pipelineLow = new PowerPlaySuperPipeline(true, false,
                        false, false, 160.0);
                webcamLow.setPipeline(pipelineLow);
                webcamLow.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                lowCameraInitialized = true;
            }

            @Override
            public void onError(int errorCode)
            {
                // This will be called if the camera could not be opened
            }
        });
        webcamLow.showFpsMeterOnViewport(false);

        while(!(lowCameraInitialized && (backCameraInitialized || frontCameraInitialized))) {
            sleep(100);
        }
        telemetry.addLine("Robot initializing, wait for completion.");
        telemetry.update();

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);

        /* Declare OpMode members. */
        robot.init(hardwareMap,false);

        telemetry.addLine("Hardware initialized...");
        telemetry.update();

        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H_A );
        sleep(300);
        performEveryLoop();

        // Perform setup needed to center turret
        robot.turretPIDPosInit( robot.TURRET_ANGLE_AUTO_R );
        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH_A );
        while( !isStopRequested() && ( robot.turretMotorPIDAuto == true || robot.liftMotorPIDAuto == true )) {
            performEveryLoop();
        }

        telemetry.addLine("Turret positioned, ready to start.");
        telemetry.update();
        createAutoStorageFolder(true, true);
        if(alignToFront) {
            pipelineFront.overrideAlliance(true);
            pipelineFront.overrideSide(true);
            pipelineFront.setStorageFolder(storageDir);
        } else {
            pipelineBack.overrideAlliance(true);
            pipelineBack.overrideSide(true);
            pipelineBack.setStorageFolder(storageDir);
        }
        pipelineLow.overrideAlliance(true);
        pipelineLow.overrideSide(true);
        pipelineLow.setStorageFolder(storageDir);

        waitForStart();
        globalCoordinatePositionReset();

        while (opModeIsActive())
        {
            performEveryLoop();
            alignToPole(alignToFront);

            int sleepCycles = 5;
            while(sleepCycles > 0) {
                sleepCycles--;
                telemetry.addLine("Aligned... waiting for kick");
                telemetry.addData("Time remaining", sleepCycles);
                telemetry.addData("Current", "X=%.1f, Y=%.1f, Angle=%.1f", currentPositionX, currentPositionY, currentPositionAngle );
                telemetry.addData(" offsets", "x=%.1f, y=%.1f", targetDistanceX, targetDistanceY );
                telemetry.addData("Odometry", "X=%.1f, Y=%.1f, Angle=%.1f", targetPositionX, targetPositionY, targetPositionAngle );
                telemetry.addData("Turret", "%.1f deg", targetAngle );
//              telemetry.addData("Max Power", maxPower);
//              telemetry.addData(" ", "#  ERR @ Pwr  (Pmin   +   P   +   I   +   D)");
//              for (int index = 0; index < LOGSIZE; index++) {
//                  telemetry.addData(" ", "%2d %.1f @ %.2f (%.2f + %.2f + %.2f + %.2f)",
//                          index, errorHistory[index], kTHistory[index],
//                          kpMinHistory[index], kpHistory[index], kiHistory[index], kdHistory[index]);
//              }
                telemetry.update();
                sleep(1000);
            }
        }
    }

    /*---------------------------------------------------------------------------------*/

    void logPid(int alignedCount, double turretPower, PIDController pid) {
        // Only save "meaningful" entries (not final 15 aligned results with all zeros!)
        if( alignedCount <= 3 ) {
            // Shift all previous instrumentation readings down one entry
            for( int index=1; index<LOGSIZE; index++ ) {
                errorHistory[index-1] = errorHistory[index];
                kpMinHistory[index-1] = kpMinHistory[index];
                kpHistory[index-1]    = kpHistory[index];
                kiHistory[index-1]    = kiHistory[index];
                kdHistory[index-1]    = kdHistory[index];
                kTHistory[index-1]    = kTHistory[index];
            }
            // Add the latest numbers to the end
            errorHistory[LOGSIZE-1] = pid.error;
            kpMinHistory[LOGSIZE-1] = (pid.error > 0)? 0.075 : -0.075;
            kpHistory[LOGSIZE-1]    = (pid.kp * pid.error);
            kiHistory[LOGSIZE-1]    = (pid.ki * pid.integralSum);
            kdHistory[LOGSIZE-1]    = (pid.kd * pid.derivative);
            kTHistory[LOGSIZE-1]    = turretPower;
        }

        telemetry.addData("turretPower: ", turretPower);
        telemetry.addData("PID", "error: %.2f, errorPwr: %.3f", pid.error, (pid.kp*pid.error) );
        telemetry.addData("PID", "integralSum: %.3f, integralSumPwr: %.3f", pid.integralSum, pid.ki*pid.integralSum);
        telemetry.addData("PID", "derivative: %.3f, derivativePwr: %.3f", pid.derivative, pid.kd*pid.derivative);
        telemetry.update();
    } // logPid

}