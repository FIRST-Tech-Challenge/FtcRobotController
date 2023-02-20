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
//      robot.turretPosRun(false);
//      robot.liftPosRun();
        robot.turretPIDPosRun(false);
        robot.liftPIDPosRun(false);
    }

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
        robot.turretPIDPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPIDPosInit( robot.LIFT_ANGLE_HIGH );
        while( !isStopRequested() && ( robot.turretMotorPIDAuto == true || robot.liftMotorPIDAuto == true )) {
            performEveryLoop();
        }
        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );

        telemetry.addLine("Turret positioned, ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            performEveryLoop();
//          robot.liftPIDPosInit( robot.LIFT_ANGLE_ASTART );
            // Execute the automatic turret movement code
            telemetry.addData("pSinLift", robot.liftPidController.ksinLift);
            telemetry.addData("pStaticLift", robot.liftPidController.kStaticLift);
            telemetry.addData("kpLift", robot.liftPidController.pidLift.kp);
            while(robot.liftMotorPIDAuto && opModeIsActive()) {
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