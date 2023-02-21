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
import static java.lang.Math.cos;
import static java.lang.Math.sin;
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
    HardwareSlimbot robot = new HardwareSlimbot();
    boolean aligning = false;
    boolean ranging = false;
    boolean turretFacingFront = false;
    boolean lowCameraInitialized = false;
    boolean frontCameraInitialized = false;
    boolean backCameraInitialized = false;
    double maxPower = 0.0;

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

        double targetAngle;
        double targetDistanceX;
        double targetDistanceY;
        double targetPositionX;
        double targetPositionY;

        // If we add back front camera, use boolean to determine which pipeline to use.
        alignmentPipeline = turretFacingFront ? pipelineFront : pipelineBack;

        alignmentPipeline.isBlueAlliance = true;
        alignmentPipeline.isLeft = true;

        // Get the image to do our decision making
        theLocalPole = alignmentPipeline.getDetectedPole();
        alignmentPipeline.savePoleAutoImage(theLocalPole);
        // This is the angle the pole is in relation to the turret angle
        targetAngle = robot.turretAngle + theLocalPole.centralOffsetDegrees;
        robot.turretPIDPosInit(targetAngle);
        targetDistanceX = theLocalPole.highDistanceOffsetCm * cos(toRadians(targetAngle));
        targetDistanceY = theLocalPole.highDistanceOffsetCm * sin(toRadians(targetAngle));
        targetPositionX = robotGlobalXCoordinatePosition + targetDistanceX;
        targetPositionY = robotGlobalYCoordinatePosition + targetDistanceY;
        driveToPosition( targetPositionY, targetPositionX, targetAngle, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_THRU );
        while(opModeIsActive() && robot.turretMotorPIDAuto) {
            performEveryLoop();
        }

        robot.stopMotion();
        robot.turretMotorSetPower(0.0);

        // Get the image after our adjustments
        theLocalPole = alignmentPipeline.getDetectedPole();
        alignmentPipeline.savePoleAutoImage(theLocalPole);
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
                    pipelineFront = new PowerPlaySuperPipeline(false, true, false, false, 144.0);
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
                    pipelineBack = new PowerPlaySuperPipeline(false, true, false, false, 144.0);
                    webcamBack.setPipeline(pipelineFront);
                    webcamBack.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    backCameraInitialized = true;
                }

                @Override
                public void onError(int errorCode) {
                    // This will be called if the camera could not be opened
                }
            });
            webcamFront.showFpsMeterOnViewport(false);
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

        while(!(lowCameraInitialized && (frontCameraInitialized || backCameraInitialized))) {
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

        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );
        sleep(300);
        performEveryLoop();

        // Perform setup needed to center turret
        robot.turretPIDPosInit( robot.TURRET_ANGLE_COLLECT_R );
        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH );
        while( !isStopRequested() && ( robot.turretMotorPIDAuto == true || robot.liftMotorPIDAuto == true )) {
            performEveryLoop();
        }

        telemetry.addLine("Turret positioned, ready to start.");
        telemetry.update();

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
                telemetry.addData("Max Power", maxPower);
                telemetry.addData(" ", "#  ERR @ Pwr  (Pmin   +   P   +   I   +   D)");
                for (int index = 0; index < LOGSIZE; index++) {
                    telemetry.addData(" ", "%2d %.1f @ %.2f (%.2f + %.2f + %.2f + %.2f)",
                            index, errorHistory[index], kTHistory[index],
                            kpMinHistory[index], kpHistory[index], kiHistory[index], kdHistory[index]);
                }
                telemetry.update();
                sleep(10000);
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