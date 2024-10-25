package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.List;

public class SimplifiedOdometryRobot {
    // Adjust these numbers to suit your robot.
    private final double ODOM_INCHES_PER_COUNT = 0.002969; // GoBilda Odometry Pod (1/226.8)
    private final boolean INVERT_DRIVE_ODOMETRY = true; // When driving FORWARD, the odometry value MUST increase. If it does not, flip the value of this constant.
    private final boolean INVERT_STRAFE_ODOMETRY = true; // When strafing to the LEFT, the odometry value MUST increase. If it does not, flip the value of this constant.

    private static final double DRIVE_GAIN = 0.03; // Strength of axial position control
    private static final double DRIVE_ACCEL = 2.0; // Acceleration limit. Percent Power change per second. 1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE = 0.5; // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND = 0.2; // Error less than this causes zero output. Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO = 0.6; // "default" Maximum Axial power limit during autonomous

    private static final double STRAFE_GAIN = 0.03; // Strength of lateral position control
    private static final double STRAFE_ACCEL = 1.5; // Acceleration limit. Percent Power change per second. 1.0 = 0-100% power in 1 sec.
    private static final double STRAFE_TOLERANCE = 0.5; // Controller is "inPosition" if position error is < +/- this amount
    private static final double STRAFE_DEADBAND = 0.2; // Error less than this causes zero output. Must be smaller than DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO = 0.6; // "default" Maximum Lateral power limit during autonomous

    private static final double YAW_GAIN = 0.018; // Strength of Yaw position control
    private static final double YAW_ACCEL = 3.0; // Acceleration limit. Percent Power change per second. 1.0 = 0-100% power in 1 sec.
    private static final double YAW_TOLERANCE = 1.0; // Controller is "inPosition" if position error is < +/- this amount
    private static final double YAW_DEADBAND = 0.25; // Error less than this causes zero output. Must be smaller than DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO = 0.6; // "default" Maximum Yaw power limit during autonomous

    // Public Members
    public double driveDistance = 0; // scaled axial distance (+ = forward)
    public double strafeDistance = 0; // scaled lateral distance (+ = left)
    public double heading = 0; // Latest Robot heading from IMU

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl driveController = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl strafeController = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public ProportionalControl yawController = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE, YAW_DEADBAND, true);

    // --- Private Members

    // Hardware interface Objects
    private Hardware hardware;

    private DcMotor driveEncoder; // the Axial (front/back) Odometry Module (may overlap with motor, or may not)
    private DcMotor strafeEncoder; // the Lateral (left/right) Odometry Module (may overlap with motor, or may not)

    private LinearOpMode myOpMode;
    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime(); // Used for any motion requiring a hold time or timeout.

    private int rawDriveOdometer = 0; // Unmodified axial odometer count
    private int driveOdometerOffset = 0; // Used to offset axial odometer
    private int rawStrafeOdometer = 0; // Unmodified lateral odometer count
    private int strafeOdometerOffset = 0; // Used to offset lateral odometer
    private double rawHeading = 0; // Unmodified heading (degrees)
    private double headingOffset = 0; // Used to offset heading

    private double turnRate = 0; // Latest Robot Turn Rate from IMU
    private boolean showTelemetry = false;

    // Robot Constructor
    public SimplifiedOdometryRobot(LinearOpMode opmode) {
        myOpMode = opmode;
        hardware = new Hardware(opmode); // Initialize hardware
    }

    /**
     * Robot Initialization:
     * Use the hardware map to Connect to devices.
     * Perform any set-up all the hardware devices.
     * @param showTelemetry Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry) {
        hardware.init(); // Initialize hardware

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        // Connect to the encoder channels using the name of that channel.
        driveEncoder = myOpMode.hardwareMap.get(DcMotor.class, "axial");
        strafeEncoder = myOpMode.hardwareMap.get(DcMotor.class, "lateral");

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // zero out all the odometry readings.
        resetOdometry();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public boolean readSensors() {
        rawDriveOdometer = driveEncoder.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
        rawStrafeOdometer = strafeEncoder.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);
        driveDistance = (rawDriveOdometer - driveOdometerOffset) * ODOM_INCHES_PER_COUNT;
        strafeDistance = (rawStrafeOdometer - strafeOdometerOffset) * ODOM_INCHES_PER_COUNT;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading = orientation.getYaw(AngleUnit.DEGREES);
        heading = rawHeading - headingOffset;
        turnRate = angularVelocity.zRotationRate;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Odom Ax:Lat", "%6d %6d", rawDriveOdometer - driveOdometerOffset, rawStrafeOdometer - strafeOdometerOffset);
            myOpMode.telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
            myOpMode.telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", heading, turnRate);
        }
        return true; // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    // ######################## Mid level control functions. #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches Distance to travel. +ve = forward, -ve = reverse.
     * @param power Maximum power to apply. This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position. 0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(distanceInches, power); // achieve desired drive distance
        strafeController.reset(0); // Maintain zero strafe drift
        yawController.reset(); // Maintain last turn heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()) {
            // implement desired axis powers
            hardware.driveRobot(driveController.getOutput(driveDistance), yawController.getOutput(heading), strafeController.getOutput(strafeDistance));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break; // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches Distance to travel. +ve = left, -ve = right.
     * @param power Maximum power to apply. This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position. 0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(0.0); // Maintain zero drive drift
        strafeController.reset(distanceInches, power); // Achieve desired Strafe distance
        yawController.reset(); // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()) {
            // implement desired axis powers
            hardware.driveRobot(driveController.getOutput(driveDistance), yawController.getOutput(heading), strafeController.getOutput(strafeDistance));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break; // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg Heading to obtain. +ve = CCW, -ve = CW.
     * @param power Maximum power to apply. This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position. 0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {
        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive() && readSensors()) {
            // implement desired axis powers
            hardware.driveRobot(0, yawController.getOutput(heading), strafeController.getOutput(strafeDistance));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break; // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    // ######################## Low level control functions. ###############################

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        hardware.setDrivePower(0, 0, 0, 0);
    }

    /**
     * Set odometry counts and distances to zero.
     */
    public void resetOdometry() {
        readSensors();
        driveOdometerOffset = rawDriveOdometer;
        driveDistance = 0.0;
        driveController.reset(0);

        strafeOdometerOffset = rawStrafeOdometer;
        strafeDistance = 0.0;
        strafeController.reset(0);
    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading() {
        return heading;
    }

    public double getTurnRate() {
        return turnRate;
    }

    /**
     * Set the drive telemetry on or off
     */
    public void showTelemetry(boolean show) {
        showTelemetry = show;
    }
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit, and a max power output.
 */
class ProportionalControl {
    double lastOutput;
    double gain;
    double accelLimit;
    double defaultOutputLimit;
    double liveOutputLimit;
    double setPoint;
    double tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition() {
        return inPosition;
    }

    public double getSetpoint() {
        return setPoint;
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }
}
