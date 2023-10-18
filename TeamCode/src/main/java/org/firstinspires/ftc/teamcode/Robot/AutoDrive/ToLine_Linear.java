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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utility.Datalogger;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * The code is structured as a LinearOpMode
 *
 * The Sensor used here can be a REV Color Sensor V2 or V3.  Make sure the white LED is turned on.
 * The sensor can be plugged into any I2C port, and must be named "sensor_color" in the active configuration.
 *
 *   Depending on the height of your color sensor, you may want to set the sensor "gain".
 *   The higher the gain, the greater the reflected light reading will be.
 *   Use the SensorColor sample in this folder to determine the minimum gain value that provides an
 *   "Alpha" reading of 1.0 when you are on top of the white line.  In this sample, we use a gain of 15
 *   which works well with a Rev V2 color sensor
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set halfway between the bare-tile, and white-line "Alpha" values.
 *   The reflected light value can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the sensor on and off the white line and note the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD halfway between the min and max.
 *
 *   Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *   Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Color: Auto Drive To Color Line", group="Robot")
//@Disabled
public class ToLine_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    // Declare OpMode members
    Datalog datalog = new Datalog("ToLine_Linear_01");
    // for data logging use in this specific OpMode since IMU is not used directly
    private IMU imu             = null;      // Control/Expansion Hub IMU
    private final DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    String[] motorLabels = {
            "motorLeftFront",           // port 0 Control Hub
            "motorLeftBack",            // port 1 Control Hub
            "motorRightFront",          // port 2 Control Hub
            "motorRightBack"            // port 3 Control Hub
    };


    /** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;

    static final double     WHITE_THRESHOLD = 0.5;  // spans between 0.0 - 1.0 from dark to light
    static final double     APPROACH_SPEED  = 0.25;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        // If necessary, turn ON the white LED (if there is no LED switch on the sensor)
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        // Some sensors allow you to set your light sensor gain for optimal sensitivity...
        // See the SensorColor sample in this folder for how to determine the optimal gain.
        // A gain of 15 causes a Rev Color Sensor V2 to produce an Alpha value of 1.0 at about 1.5" above the floor.
        colorSensor.setGain(15);

        // Wait for driver to press PLAY)
        // Abort this loop is started or stopped.
        while (opModeInInit()) {

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to drive to white line.");    //

            // Display the light level while we are waiting to start
            getBrightness();
        }

        // Start the robot moving forward, and then begin looking for a white line.
        //leftDrive.setPower(APPROACH_SPEED);
        //rightDrive.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (getBrightness() < WHITE_THRESHOLD)) {
            sleep(5);
        }

        // Stop all motors
        setMotorSpeed(0.0);  // stop all motors
    }

    // to obtain reflected light, read the normalized values from the color sensor.  Return the Alpha channel.
    double getBrightness() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
        telemetry.update();

        return colors.alpha;
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
