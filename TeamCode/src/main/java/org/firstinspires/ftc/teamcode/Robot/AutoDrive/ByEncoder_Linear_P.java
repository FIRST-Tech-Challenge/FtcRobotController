/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Robot.AutoDrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  The equation to reduce speed of motor travel to a preset linear distance for travel under proportional control is:
 *
 * speed = Kp * (target_distance - current_distance)
 *
 * Where:
 *
 *     Kp is the proportional gain constant
 *     target_distance is the desired distance to travel
 *     current_distance is the current distance traveled
 *
 * The proportional gain constant, Kp, determines how sensitive the motor speed is to the difference between the target distance and the current distance. A higher Kp value will cause the motor speed to change more quickly in response to a difference in distance, while a lower Kp value will cause the motor speed to change more slowly.
 *
 *  Adapted from SDK 8.2 example, RobotAutoDriveByEncoderLinear
 *  2023-08-01 0.1 armw initial foray using prior season's Mecanum drivetrain parameters
 */

@Autonomous(name="Auto: Drive By Encoder with P", group="Robot")
//@Disabled
public class ByEncoder_Linear_P extends LinearOpMode {

    // Declare OpMode members
    Datalog datalog = new Datalog("ByEncoder_Linear_P_01");
    // for data logging use in this specific OpMode since IMU is not used directly
    private IMU imu             = null;      // Control/Expansion Hub IMU
    private final double robotHeading  = 0;
    private final double headingError  = 0;
    private final double targetHeading = 0;

    private final DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    String[] motorLabels = {
            "motorLeftFront",           // port 0 Control Hub
            "motorLeftBack",            // port 1 Control Hub
            "motorRightFront",          // port 2 Control Hub
            "motorRightBack"            // port 3 Control Hub
    };

    private final ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.6898396 ; // 5203-2402-0019  goBILDA 5203 series 19.2:1 ratio
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;         // already applied to COUNTS_PER_MOTOR_REV
    static final double     WHEEL_DIAMETER_INCHES   = 3.779528 ;    // 96 mm Mecanum wheel
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables
        int inchesLeft, inchesRight;
        // Initialize the hardware variables
        // define initialization values for IMU, and then initialize it.
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters;

        new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        // The strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        for (int i = 0; i < motor.length; i++) {
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);
            // motor stops and then brakes actively resisting any external force which attempts to turn the motor
            motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motor[0].setDirection(DcMotorEx.Direction.REVERSE); // motorLeftFront
        motor[1].setDirection(DcMotorEx.Direction.REVERSE); // motorLeftBack
        motor[2].setDirection(DcMotorEx.Direction.FORWARD); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.FORWARD); // motorRightBack

        inchesLeft = 60;
        inchesRight = inchesLeft;
        telemetry.addData("Travel length", "%4d", inchesLeft);
        telemetry.addData("Starting at",  "%7d :%7d",
            motor[0].getCurrentPosition(), motor[2].getCurrentPosition());
        telemetry.addLine("Start: press PLAY");
        telemetry.update();

        // data logging for offline analysis
        datalog.ticks0.set(motor[0].getCurrentPosition());
        datalog.ticks0.set(motor[1].getCurrentPosition());
        datalog.ticks0.set(motor[2].getCurrentPosition());
        datalog.ticks0.set(motor[3].getCurrentPosition());
        orientation = imu.getRobotYawPitchRollAngles();
        datalog.heading.set(orientation.getYaw(AngleUnit.DEGREES));
        datalog.writeLine();    // timestamp applied by helper class

        while (!opModeIsActive() && !isStopRequested()) {
            sleep(50);
            idle();
        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  inchesLeft,  inchesRight, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(3000);  // pause to let driver absorb the message
    }

    /**
     * helper method to set a common speed for all motors in the drivetrain
     * @param speed setting applied to all motors
     */
    private void setMotorSpeed(double speed) {
        for (DcMotorEx dcMotorEx : motor) {
            dcMotorEx.setPower(speed);
        }
    }

    /**
     * helper method to set a common speed for all motors in the drivetrain
     * @param mode setting applied to all motors
     */
    private void setMotorMode(DcMotorEx.RunMode mode) {
        for (DcMotorEx dcMotorEx : motor) {
            //dcMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dcMotorEx.setMode(mode);
        }
    }

    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running
     * @param speed initial speed of motors
     * @param leftInches linear travel distance for left side motors
     * @param rightInches linear travel distance for right side motors
     * @param timeoutS pause period after reaching target linear travel distance
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        final int[] target = new int[]{0, 0, 0, 0};

        if (opModeIsActive()) { // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            target[0] = motor[0].getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            target[1] = motor[1].getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            target[2] = motor[2].getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            target[3] = motor[3].getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motor[0].setTargetPosition(target[0]);
            motor[1].setTargetPosition(target[1]);
            motor[2].setTargetPosition(target[2]);
            motor[3].setTargetPosition(target[3]);
            datalog.distance.set(target[0]);
            datalog.setSpeed.set(speed);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn On RUN_TO_POSITION

            runtime.reset(); // reset the timeout time and start motion
            setMotorSpeed(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (motor[0].isBusy() && motor[2].isBusy() && motor[2].isBusy() && motor[3].isBusy())) {

                                        // Display current parameters to the driver
                telemetry.addData("Running to",  " %7d :%7d", target[0], target[1]);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            motor[0].getCurrentPosition(), motor[1].getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d",
                        motor[2].getCurrentPosition(), motor[3].getCurrentPosition());
                telemetry.update();
                                        // data logging for offline analysis
                datalog.ticks0.set(motor[0].getCurrentPosition());
                datalog.ticks0.set(motor[1].getCurrentPosition());
                datalog.ticks0.set(motor[2].getCurrentPosition());
                datalog.ticks0.set(motor[3].getCurrentPosition());
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                datalog.heading.set(orientation.getYaw(AngleUnit.DEGREES));
                datalog.writeLine();    // timestamp applied by helper class
            }

            setMotorSpeed(0.0);         // Stop all motion
            setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // Turn off RUN_TO_POSITION

            sleep(10000);   // optional pause after each move.
        }
    }

    /**
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus     = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField setSpeed         = new Datalogger.GenericField("Initial Speed");
        public Datalogger.GenericField direction        = new Datalogger.GenericField("Direction");
        public Datalogger.GenericField distance         = new Datalogger.GenericField("Distance");
        public Datalogger.GenericField ticks0           = new Datalogger.GenericField("Ticks 0");
        public Datalogger.GenericField ticks1           = new Datalogger.GenericField("Ticks 1");
        public Datalogger.GenericField ticks2           = new Datalogger.GenericField("Ticks 2");
        public Datalogger.GenericField ticks3           = new Datalogger.GenericField("Ticks 3");
        public Datalogger.GenericField currentSpeed0    = new Datalogger.GenericField("Current Speed 0");
        public Datalogger.GenericField currentSpeed1    = new Datalogger.GenericField("Current Speed 1");
        public Datalogger.GenericField currentSpeed2    = new Datalogger.GenericField("Current Speed 2");
        public Datalogger.GenericField currentSpeed3    = new Datalogger.GenericField("Current Speed 3");
        public Datalogger.GenericField heading          = new Datalogger.GenericField("Heading");

        /**
         * Initializer for class
         * @param name filename for output log
         */
        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            setSpeed,
                            direction,
                            distance,
                            ticks0,
                            ticks1,
                            ticks2,
                            ticks3,
                            currentSpeed0,
                            currentSpeed1,
                            currentSpeed2,
                            currentSpeed3,
                            heading
                    )
                    .build();
        }

        /**
         * Tell the datalogger to gather the values of the fields
         * and write a new line in the log.
         */
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
