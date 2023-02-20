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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
//@Disabled
public class PoleOrientationExample extends LinearOpMode
{
    final int LOGSIZE = 12;
    double[]  errorHistory = new double[LOGSIZE];
    double[]  kpMinHistory = new double[LOGSIZE];
    double[]  kpHistory    = new double[LOGSIZE];
    double[]  kiHistory    = new double[LOGSIZE];
    double[]  kdHistory    = new double[LOGSIZE];
    double[]  kTHistory    = new double[LOGSIZE];

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
        robot.turretPIDPosRun(false);
        robot.liftPIDPosRun(false);
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
//      robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
        robot.turretPIDPosInit(+32.5 );
        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH_BA );
        while( robot.turretMotorPIDAuto == true || robot.liftMotorPIDAuto == true) {
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

            // Use the camera to align to the pole, and set the correct distance away
            alignToPole(false);
            telemetry.addLine("Aligned... waiting for kick");
            telemetry.addData(" ","#  ERR @ Pwr  (Pmin   +   P   +   I   +   D)");
            for( int index=0; index<LOGSIZE; index++ ) {
                telemetry.addData(" ","%2d %.1f @ %.2f (%.2f + %.2f + %.2f + %.2f)",
                        index, errorHistory[index], kTHistory[index],
                        kpMinHistory[index], kpHistory[index], kiHistory[index], kdHistory[index] );
            }
            telemetry.update();
            sleep( 5000 );
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

    void logPid(int alignedCount, double turretPower, PIDController pid) {
        // Only save "meaningful" entries (not final 8 aligned results with all zeros!)
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

    void alignToPole(boolean turretFacingFront) {
        PowerPlaySuperPipeline alignmentPipeline;
        PowerPlaySuperPipeline.AnalyzedPole theLocalPole;
        final double DRIVE_SLOPE  = 0.004187;
        final double DRIVE_OFFSET = 0.04522;
        final int TURRET_CYCLES_AT_POS = 8;
        // minPower=0; kp = 0.0027
        // These values are no longer valid switching to angle
        PIDControllerTurret pidController = new PIDControllerTurret(0.00008,0.000, 0.00010, 0.075);

        double turretPower;
        double turretPowerMax = 0.14;  // maximum we don't want the PID to exceed
        double drivePower;

        // If we add back front camera, use boolean to determine which pipeline to use.
//        alignmentPipeline = turretFacingFront ? pipelineFront : pipelineBack;
        alignmentPipeline = pipelineBack;

        theLocalPole = alignmentPipeline.getDetectedPole();
        while (opModeIsActive() && ((theLocalPole.alignedCount <= TURRET_CYCLES_AT_POS) ||
                theLocalPole.properDistanceHighCount <= 3)) {
            performEveryLoop();
            turretPower = pidController.update(0.0, theLocalPole.centralOffsetDegrees);
            // Ensure we never exceed a safe power
            if( turretPower > +turretPowerMax ) turretPower = +turretPowerMax;
            if( turretPower < -turretPowerMax ) turretPower = -turretPowerMax;
            logPid(theLocalPole.alignedCount, turretPower, pidController);

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
                robot.turretMotorSetPower(0);
            } else {
                driveAndRotateTurretAngle(drivePower, turretPower, turretFacingFront);
            }

            theLocalPole = alignmentPipeline.getDetectedPole();
        }
        robot.stopMotion();
        robot.turretMotorSetPower(0.0);
    } // alignToPole

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
        robot.turretMotorSetPower(turretPower);
    } // driveAndRotateTurretAngle
}