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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple poles, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@TeleOp(name="Arm-Test", group="Skunkworks")
//@Disabled
public class ArmPIDTester extends LinearOpMode
{
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
    boolean liftFacingFront = false;
    boolean lowCameraInitialized = false;
    boolean backCameraInitialized = false;
    double maxPower = 0.0;

    /**
     * NOTE: Many comments have been omitted from this sample for the
     * sake of conciseness. If you're just starting out with EasyOpenCv,
     * you should take a look at or its
     * webcam counterpart,first.
     */

    public void performEveryLoop() {
        robot.readBulkData();
        robot.turretPosRun(false);
        robot.liftPosRun();
        liftPIDPosRun(true);
    }

    boolean liftMotorPIDAuto = false;
    boolean liftMotorLogging = false;
    boolean liftMotorLogEnable = false;
    public final static int LIFTMOTORLOG_SIZE  = 128;   // 128 entries = 2+ seconds @ 16msec/60Hz
    double liftAngleTarget;
    int liftMotorCycles = 0;
    int liftMotorWait   = 0;
    int liftMotorLogIndex = 0;
    protected double[]      liftMotorLogTime   = new double[LIFTMOTORLOG_SIZE];  // msec
    protected double[]      liftMotorLogAngle  = new double[LIFTMOTORLOG_SIZE];  // Angle [degrees]
    protected double[]      liftMotorLogPwr    = new double[LIFTMOTORLOG_SIZE];  // Power
    protected double[]      liftMotorLogAmps   = new double[LIFTMOTORLOG_SIZE];  // mAmp
    protected ElapsedTime liftMotorTimer     = new ElapsedTime();
    /*--------------------------------------------------------------------------------------------*/
    public void writeLiftLog() {
        // Are we even logging these events?
        if( !liftMotorLogging) return;
        // Movement must be complete (disable further logging to memory)
        liftMotorLogEnable = false;
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "//FIRST//TurretMotor//" + dateString;
        // Ensure that directory path exists
        File directory = new File(directoryPath);
        directory.mkdirs();
        // Create a filename based on TIME
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String filePath = directoryPath + "/" + "lift_" + timeString + ".txt";
        // Open the file
        FileWriter liftLog;
        try {
            liftLog = new FileWriter(filePath, false);
            liftLog.write("LiftMotor\r\n");
            liftLog.write("Target Angle," + liftAngleTarget + "\r\n");
            // Log Column Headings
            liftLog.write("msec,pwr,mAmp,angle\r\n");
            // Log all the data recorded
            for( int i=0; i<liftMotorLogIndex; i++ ) {
                String msecString = String.format("%.3f, ", liftMotorLogTime[i] );
                String pwrString  = String.format("%.3f, ", liftMotorLogPwr[i]  );
                String ampString  = String.format("%.0f, ", liftMotorLogAmps[i] );
                String degString  = String.format("%.2f\r\n", liftMotorLogAngle[i]  );
                liftLog.write( msecString + pwrString + ampString + degString );
            }
            liftLog.flush();
            liftLog.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    } // writeLiftLog()

    /**
     * @param p1 Power required to almost move
     * @param v1 Voltage at which power was applied in mV
     * @param p2 Power required to almost move
     * @param v2 Votlage at which power was applied in mV
     * @return Linear interpolated value
     */
    public double getInterpolatedMinPower(double p1, double v1, double p2, double v2) {
        double result;
        double voltage = robot.readBatteryExpansionHub();
        double slope = (p1 - p2) / (v1 - v2);
        result = slope * (voltage - v2) + p2;

        return result;
    }

    /*--------------------------------------------------------------------------------------------*/
    /* liftPosInit()                                                                            */
    /* - newAngle = desired lift angle                                                          */
    public void liftPIDPosInit( double newAngle )
    {
        // Current distance from target (degrees)
        double degreesToGo = newAngle - robot.liftAngle;
        double pSinLift = 0.007;
        double pStaticLift = 0.320;
        double pSinLower = 0.007;
        double pStaticLower = 0.110;
        // Voltage doesn't seem as important on the arm minimum. Probably don't have to do
        // interpolated voltage. For example 0.13 power was not able to move arm at low voltage
        // and also could not at fresh battery voltage. 0.131 was able to at low voltage.
        // pStaticLower 0.130 @ 12.54V
        // pStaticLift 0.320 @ 12.81V
//        double pSin = getInterpolatedMinPower();

//        pidController = new PIDControllerWormArm(-0.04, 0.000, 0.000,
//                pSin, pStatic);
        pidController = new PIDControllerWormArm(-0.1, 0.000, -0.007,
                pSinLift, pStaticLift, -0.030, 0.000, -0.007, pSinLower, pStaticLower);

        // Are we ALREADY at the specified angle?
        if( Math.abs(degreesToGo) <= 1.0 )
            return;

        pidController.reset();

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        robot.liftMotorsSetPower( 0.0 );

        // Establish a new target angle & reset counters
        liftMotorPIDAuto = true;
        liftAngleTarget = newAngle;
        liftMotorCycles = 0;
        liftMotorWait   = 0;

        // If logging instrumentation, begin a new dataset now:
        if( liftMotorLogging ) {
            liftMotorLogIndex  = 0;
            liftMotorLogEnable = true;
            liftMotorTimer.reset();
        }

    } // liftPosInit

    // pStaticLower = 0.0 @ V
    // pStaticLower = 0.0 @ V
    PIDControllerWormArm pidController;

    /*--------------------------------------------------------------------------------------------*/
    /* liftPosRun()                                                                             */
    public void liftPIDPosRun( boolean teleopMode )
    {
        // Has an automatic movement been initiated?
        if(liftMotorPIDAuto) {
            // Keep track of how long we've been doing this
            liftMotorCycles++;
            // Current distance from target (angle degrees)
            double degreesToGo = liftAngleTarget - robot.liftAngle;
            double degreesToGoAbs = Math.abs(degreesToGo);
            int waitCycles = (teleopMode) ? 5 : 2;
            double power = pidController.update(liftAngleTarget, robot.liftAngle);
            telemetry.addData("Set Power", power);
            telemetry.addData("p", pidController.pidCurrent.kp);
            telemetry.addData("i", pidController.pidCurrent.ki);
            telemetry.addData("d", pidController.pidCurrent.kd);
            telemetry.addData("sin", pidController.ksinCurrent);
            telemetry.addData("static", pidController.kStaticCurrent);
            telemetry.addData("error", pidController.pidCurrent.error);
            telemetry.addData("target", liftAngleTarget);
            telemetry.addData("state", robot.liftAngle);
            telemetry.update();
            robot.liftMotorsSetPower(power);
            if(abs(power) > abs(maxPower)) {
                maxPower = power;
            }
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
            if( degreesToGoAbs <= 1.0 ) {
                if( ++liftMotorWait >= waitCycles ) {
                    liftMotorPIDAuto = false;
                    robot.liftMotorsSetPower(0);
                    writeLiftLog();
                }
            }
        } // liftMotorAuto
    } // liftPosRun

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Robot initializing, wait for completion.");
        telemetry.update();

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);

        /* Declare OpMode members. */
        robot.init(hardwareMap,false);

        telemetry.addLine("Hardware initialized...");
        telemetry.update();

        robot.grabberSetTilt( robot.GRABBER_TILT_STORE );
        sleep(300);
        performEveryLoop();

        // Perform setup needed to center lift
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPosInit( robot.LIFT_ANGLE_HIGH );
        while( !isStopRequested() && ( robot.turretMotorAuto == true || robot.liftMotorAuto == true )) {
            performEveryLoop();
        }
        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );

        telemetry.addLine("Turret positioned, ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            performEveryLoop();
            liftPIDPosInit( robot.LIFT_ANGLE_ASTART );
            // Execute the automatic turret movement code
            telemetry.addData("pSinLift", pidController.ksinLift);
            telemetry.addData("pStaticLift", pidController.kStaticLift);
            telemetry.addData("kpLift", pidController.pidLift.kp);
            while(liftMotorPIDAuto && opModeIsActive()) {
                performEveryLoop();
            }

            // Use the camera to align to the pole, and set the correct distance away
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
                sleep(1000);
            }
        }
    }

    /*---------------------------------------------------------------------------------*/

    void logPid(int alignedCount, double liftPower, PIDController pid) {
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
            kTHistory[LOGSIZE-1]    = liftPower;
        }

        telemetry.addData("turretPower: ", liftPower);
        telemetry.addData("PID", "error: %.2f, errorPwr: %.3f", pid.error, (pid.kp*pid.error) );
        telemetry.addData("PID", "integralSum: %.3f, integralSumPwr: %.3f", pid.integralSum, pid.ki*pid.integralSum);
        telemetry.addData("PID", "derivative: %.3f, derivativePwr: %.3f", pid.derivative, pid.kd*pid.derivative);
        telemetry.update();
    } // logPid

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
}