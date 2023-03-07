/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package org.firstinspires.ftc.teamcode.android.OpModes;

import android.util.Log;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.android.navx_ftc.src.main.java.com.kauailabs.navx.ftc.navXPIDController;

import java.text.DecimalFormat;

/*
 * An example linear op mode where the robot will drive in
 * a straight line (where the driving direction is guided by
 * the Yaw angle from a navX-Model device).
 *
 * This example uses a simple PID controller configuration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.  This example uses
 * the default update rate (50Hz), which may be lowered in order
 * to reduce the frequency of the updates to the drive system.
 */

@TeleOp(name = "Concept: navX Drive Straight PID - Linear", group = "Concept")
@Disabled
public class ConceptNavXDriveStraightPIDLinearOp extends LinearOpMode {

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        frontright = hardwareMap.get(DcMotor.class, "frontright");



        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */
        //leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                                    navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        waitForStart();

        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        navx_device.zeroYaw();

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        final double TOTAL_RUN_TIME_SECONDS = 10.0;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        /* Drive straight forward at 1/2 of full drive speed */
        double drive_speed = 0.5;

        DecimalFormat df = new DecimalFormat("#.##");

        try {
            while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        frontleft.setPower(drive_speed);
                        frontright.setPower(drive_speed);
                        backleft.setPower(drive_speed);
                        backright.setPower(drive_speed);
                        telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
                                df.format(drive_speed));
                    } else {
                        double output = yawPIDResult.getOutput();
                        frontright.setPower(drive_speed + output);
                        frontleft.setPower(drive_speed - output);
                        backleft.setPower(drive_speed - output);
                        backright.setPower(drive_speed + output);
                        telemetry.addData("PIDOutput", df.format(limit(drive_speed + output)) + ", " +
                                df.format(limit(drive_speed - output)));
                    }
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                } else{
			        /* A timeout occurred */
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        finally {
            navx_device.close();
            telemetry.addData("LinearOp", "Complete");
        }
    }
}
