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
@TeleOp(name="Turret-Test", group="Skunkworks")
//@Disabled
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
        robot.turretPIDPosRun(false);
        robot.liftPIDPosRun( false );
    }

    boolean turretMotorPIDAuto = false;

    ElapsedTime intakeTimer = new ElapsedTime();
    /*--------------------------------------------------------------------------------------------*/
    private void scoreCone() {

        // Start ejecting the cone
        intakeTimer.reset();
        // Wait for sensor to indicate it's clear (or we timeout)
        while( opModeIsActive() ) {
            performEveryLoop();
            // Ensure we eject for at least 250 msec before using sensor (in case sensor fails)
            boolean bottomSensorClear = robot.bottomConeSensor.getState() && (intakeTimer.milliseconds() > 250);
            // Also have a max timeout in case sensor fails
            boolean maxEjectTimeReached = (intakeTimer.milliseconds() >= 400);
            // Is cycle complete?
            if( bottomSensorClear || maxEjectTimeReached) break;
        }
        // Stop the ejector
        robot.grabberSetTilt( robot.GRABBER_TILT_GRAB3 );

    } // scoreCone

    public void performAutoOldLift() {
        // Now reverse the lift to raise off the cone stack
        robot.liftPosInit( robot.LIFT_ANGLE_5STACK );
        while( opModeIsActive() && (robot.liftMotorAuto == true) ) {
            performEveryLoop();
        }
        // halt lift motors
        robot.liftMotorsSetPower( 0.0 );

        // Perform setup to center turret and raise lift to scoring position
        robot.turretPosInit( robot.TURRET_ANGLE_5STACK_L);
        robot.liftPosInit( robot.LIFT_ANGLE_HIGH_BA );
        robot.grabberSetTilt( robot.GRABBER_TILT_BACK_H );
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_DOWN );
        while( opModeIsActive() && ( robot.turretMotorAuto == true || robot.liftMotorAuto == true )) {
            performEveryLoop();
        }

        scoreCone();

        robot.turretPosInit( robot.TURRET_ANGLE_CENTER );
        robot.liftPosInit( robot.LIFT_ANGLE_5STACK );
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_UP );
        while( opModeIsActive() && ((robot.turretMotorAuto == true) || (robot.liftMotorAuto == true)) ) {
            performEveryLoop();
        }
        robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
        while( opModeIsActive() && ((robot.liftMotorAuto == true)) ) {
            performEveryLoop();
        }
    }

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

    public void performTeleOldLift(double collectAngle, double scoreAngle) {
        // Now reverse the lift to raise off the cone stack
        // Perform setup to center turret and raise lift to scoring position
        robot.turretPosInit( scoreAngle );
        robot.liftPosInit( robot.LIFT_ANGLE_HIGH_B );
        robot.grabberSetTilt( robot.GRABBER_TILT_BACK_H );
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_DOWN );
        while( opModeIsActive() && ( robot.turretMotorAuto == true || robot.liftMotorAuto == true )) {
            performEveryLoop();
        }

        scoreCone();

        robot.turretPosInit( collectAngle );
        robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
        robot.rotateServo.setPosition( robot.GRABBER_ROTATE_UP );
        while( opModeIsActive() && ((robot.turretMotorAuto == true) || (robot.liftMotorAuto == true)) ) {
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

    @Override
    public void runOpMode() throws InterruptedException {
        double oldTime;
        double newTime;
        double target = 20.0;
        ElapsedTime oldWay = new ElapsedTime();
        ElapsedTime newWay = new ElapsedTime();
        telemetry.addLine("Robot initializing, wait for completion.");
        telemetry.update();

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);

        /* Declare OpMode members. */
        robot.init(hardwareMap,false);

        telemetry.addLine("Hardware initialized...");
        telemetry.update();

        robot.grabberSetTilt( robot.GRABBER_TILT_GRAB3 );
        sleep(300);
        performEveryLoop();

        // Perform setup needed to center turret
        robot.turretPosInit( 21.9 );
        robot.liftPosInit( robot.LIFT_ANGLE_COLLECT );
        while( !isStopRequested() && ( robot.turretMotorAuto == true || robot.liftMotorAuto == true )) {
            performEveryLoop();
        }

        telemetry.addLine("Turret positioned, ready to start.");
        telemetry.update();

        waitForStart();

        oldWay.reset();
        // Right
        //performTeleOldLift( 21.9, -50.5 );
        // Left
        //performTeleOldLift ( -21.9, 50.5 );
        oldTime = oldWay.milliseconds();

        newWay.reset();
        // Right
        performTeleNewLift( 21.9, 129.5 );
        // Left
        //performTeleNewLift ( -21.9, -129.5 );
        newTime = newWay.milliseconds();

        telemetry.addData("Old Timer", oldTime);
        telemetry.addData("New Timer", newTime);
        telemetry.update();
        sleep(10000);

        while (opModeIsActive())
        {
            performEveryLoop();
            robot.turretPIDPosInit(robot.TURRET_ANGLE_CENTER);
            // Execute the automatic turret movement code
            telemetry.addData("pStatic", robot.turretPidController.kStatic);
            telemetry.addData("kp", robot.turretPidController.kp);
            while(turretMotorPIDAuto && opModeIsActive()) {
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