/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Reno.poc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Reno.HardwareRobot;

@Autonomous(name="POC: Auto Drive Calibration", group="Concept")
//@Disabled
public class ConceptAutoDriveCalibration extends LinearOpMode {

    /* Declare OpMode members. */
    private HardwareRobot robot   = new HardwareRobot();
    private double  targetHeading = 0;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.5;     // Max driving speed for better distance accuracy.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

       // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            //telemetry.addData("gyro Cali status", imu.isGyroCalibrated());
            //telemetry.addData("sys Cali status", imu.isSystemCalibrated());
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.addData(">", "Robot Roll = %4.0f", getRawRoll());
            telemetry.addData(">", "Robot Pitch = %4.0f", getRawPitch());
            telemetry.update();
        }
        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        driveStraight(DRIVE_SPEED, 24.0 * 2);
        turnHeading(-90.0 );
        driveStraight(DRIVE_SPEED, 24.0);
        turnHeading( -90);
        driveStraight(DRIVE_SPEED, 24.0 * 2);
        turnHeading( -90);
        driveStraight(DRIVE_SPEED, 24.0);
        turnHeading( -90);

        sleep(1000);  // Pause to display last telemetry message.
    }


    public void driveStraight(double drivePower, double distance) {

        robot.resetEncoder();
        robot.enableEncoder();

        targetHeading = robot.getRawHeading();

        if(distance < 0)
        {
            robot.setDriveBackward();
        }
        else
        {
            robot.setDriveForward();
        }

        if (opModeIsActive()) {

            robot.setTargetPosition(Math.abs(distance));
            robot.drive(drivePower, 0);

            sendTelemetry(HardwareRobot.RobotAction.DRIVE_STRAIGHT);

            ConceptTurnPidController pid = new ConceptTurnPidController(targetHeading, 0.01, 0, 0.003);

            while (opModeIsActive() && robot.isBusyDriving())
            {

                if(Math.abs(targetHeading - robot.getRawHeading()) > HEADING_THRESHOLD)
                {
                    sendTelemetry(HardwareRobot.RobotAction.DRIVE_TURN);
                    // Determine required steering to keep on heading
                    double turnPower = pid.getValue(robot.getRawHeading());

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        turnPower *= -1.0;

                    // Apply the turning correction to the current driving speed.
                    robot.drive(drivePower, turnPower);
                }
                else
                {
                    robot.drive(drivePower, 0);
                }

            }
            robot.stop();
        }
    }

    public void turnHeading(double angle)
    {
        turnToHeading(angle+ robot.getRawHeading());
    }

    public void turnToHeading(double targetAngle) {

        robot.setDriveForward();
        robot.disableEncoder();
        targetHeading = targetAngle;
        ConceptTurnPidController pid = new ConceptTurnPidController(targetAngle, 0.01, 0, 0.003);

        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - robot.getRawHeading()) > HEADING_THRESHOLD) {
            double turnPower = pid.getValue(robot.getRawHeading());
            //robot.setMotorPower(-turnPower, turnPower, -turnPower, turnPower);
            robot.drive(0, turnPower);

            sendTelemetry(HardwareRobot.RobotAction.TURN);
        }
        robot.stop();
    }


    private void sendTelemetry(HardwareRobot.RobotAction action)
    {
        telemetry.addData("Motion", action);
        switch (action) {
            case DRIVE_STRAIGHT:
            case DRIVE_TURN:
                telemetry.addData("", robot.getTargetPosition());
                break;
            case TURN:
                telemetry.addData("Target Angle:", targetHeading);
                break;

        }
        telemetry.addData("", robot.getCurrentPosition());
        telemetry.addData("", robot.getMotorStatus());
        telemetry.addData("", robot.getRawHeading());

        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        return robot.getRawHeading();
    }

    public double getRawRoll() {
       return robot.getRawRoll();
    }

    public double getRawPitch() {
        return robot.getRawPitch();
    }

}
