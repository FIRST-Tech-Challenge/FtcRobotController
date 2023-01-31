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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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
@TeleOp(name="Pole-Test", group="Skunkworks")
@Disabled
public class TurretPIDTester extends LinearOpMode
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
        robot.turretPosRun(false);
        robot.liftPosRun();
        turretPIDPosRun(true);
    }

    boolean turretMotorAuto = false;
    boolean turretMotorLogging = false;
    boolean turretMotorLogEnable = false;
    public final static int TURRETMOTORLOG_SIZE  = 128;   // 128 entries = 2+ seconds @ 16msec/60Hz
    double turretAngleTarget;
    int turretMotorCycles = 0;
    int turretMotorWait   = 0;
    int turretMotorLogIndex = 0;
    protected double[]      turretMotorLogTime   = new double[TURRETMOTORLOG_SIZE];  // msec
    protected double[]      turretMotorLogAngle  = new double[TURRETMOTORLOG_SIZE];  // Angle [degrees]
    protected double[]      turretMotorLogPwr    = new double[TURRETMOTORLOG_SIZE];  // Power
    protected double[]      turretMotorLogAmps   = new double[TURRETMOTORLOG_SIZE];  // mAmp
    protected ElapsedTime turretMotorTimer     = new ElapsedTime();
    /*--------------------------------------------------------------------------------------------*/
    public void writeTurretLog() {
        // Are we even logging these events?
        if( !turretMotorLogging) return;
        // Movement must be complete (disable further logging to memory)
        turretMotorLogEnable = false;
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "//FIRST//TurretMotor//" + dateString;
        // Ensure that directory path exists
        File directory = new File(directoryPath);
        directory.mkdirs();
        // Create a filename based on TIME
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String filePath = directoryPath + "/" + "turret_" + timeString + ".txt";
        // Open the file
        FileWriter turretLog;
        try {
            turretLog = new FileWriter(filePath, false);
            turretLog.write("TurretMotor\r\n");
            turretLog.write("Target Angle," + turretAngleTarget + "\r\n");
            // Log Column Headings
            turretLog.write("msec,pwr,mAmp,angle\r\n");
            // Log all the data recorded
            for( int i=0; i<turretMotorLogIndex; i++ ) {
                String msecString = String.format("%.3f, ", turretMotorLogTime[i] );
                String pwrString  = String.format("%.3f, ", turretMotorLogPwr[i]  );
                String ampString  = String.format("%.0f, ", turretMotorLogAmps[i] );
                String degString  = String.format("%.2f\r\n", turretMotorLogAngle[i]  );
                turretLog.write( msecString + pwrString + ampString + degString );
            }
            turretLog.flush();
            turretLog.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    } // writeTurretLog()

    /*--------------------------------------------------------------------------------------------*/
    /* turretPosInit()                                                                            */
    /* - newAngle = desired turret angle                                                          */
    public void turretPIDPosInit( double newAngle )
    {
        // Current distance from target (degrees)
        double degreesToGo = newAngle - robot.turretAngle;

        // Are we ALREADY at the specified angle?
        if( Math.abs(degreesToGo) < 1.0 )
            return;

        pidController.reset();

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        robot.turretMotor.setPower( 0.0 );

        // Establish a new target angle & reset counters
        turretMotorAuto   = true;
        turretAngleTarget = newAngle;
        turretMotorCycles = 0;
        turretMotorWait   = 0;

        // If logging instrumentation, begin a new dataset now:
        if( turretMotorLogging ) {
            turretMotorLogIndex  = 0;
            turretMotorLogEnable = true;
            turretMotorTimer.reset();
        }

    } // turretPosInit

    PIDControllerTurret pidController = new PIDControllerTurret(0.00, 0.00, 0.00,
            0.08, 0.0);
    /*--------------------------------------------------------------------------------------------*/
    /* turretPosRun()                                                                             */
    public void turretPIDPosRun( boolean teleopMode )
    {
        // Has an automatic movement been initiated?
        if( turretMotorAuto ) {
            // Keep track of how long we've been doing this
            turretMotorCycles++;
            // Current distance from target (angle degrees)
            double degreesToGo = turretAngleTarget - robot.turretAngle;
            double degreesToGoAbs = Math.abs(degreesToGo);
            int waitCycles = (teleopMode) ? 5 : 2;
            double power = pidController.update(turretAngleTarget, robot.turretAngle);
            robot.turretMotor.setPower(power);
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
            if( degreesToGoAbs < 1.0 ) {
                if( ++turretMotorWait >= waitCycles ) {
                    turretMotorAuto = false;
                    robot.turretMotor.setPower(0);
                    writeTurretLog();
                }
            }
        } // turretMotorAuto
    } // turretPosRun

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

        // Perform setup needed to center turret
        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPosInit( robot.LIFT_ANGLE_HIGH );
        while( robot.turretMotorAuto == true || robot.liftMotorAuto == true) {
            performEveryLoop();
        }
        robot.grabberSetTilt( robot.GRABBER_TILT_FRONT_H );

        telemetry.addLine("Turret positioned, ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            turretPIDPosInit(robot.TURRET_ANGLE_CENTER);
            // Execute the automatic turret movement code
            while(turretMotorAuto) {
                performEveryLoop();
            }

            // Use the camera to align to the pole, and set the correct distance away
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