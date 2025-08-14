package org.firstinspires.ftc.teamcode.subsystems.odometry;

import android.widget.GridLayout;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Specifications;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

/**
 * PinPointOdo class handles odometry and pose estimation using a GoBildaPinpointDriver sensor.
 * It manages position (x, y), heading, velocities, and dead reckoning fallback if sensor readings are invalid.
 */
public class PinPointOdometrySubsystem {
    // Underlying odometry driver instance
    private GoBildaPinpointDriver pinpointDriver;

    // Current pose estimates (in cm or degrees as appropriate)
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // Previous pose estimates (used for dead reckoning if sensor data invalid)
    private double previousX = 0;
    private double previousY = 0;
    private double previousHeading = 0;

    // Current velocities in each direction
    private double vx = 0;
    private double vy = 0;
    private double vtheta = 0;

    // Timer to track the control loop interval
    private ElapsedTime controllerLoopTime;

    // Counter for number of NaN (invalid) sensor readings
    private int nanCounter = 0;

    /**
     * Constructor initializes the PinPointOdo with hardware mapping, sets encoder parameters and resets sensor.
     * @param hw Hwardware to access sensors
     */
    public PinPointOdometrySubsystem(Hardware hw){
        // Get the GoBildaPinpointDriver from hardware map with configured name
        pinpointDriver = hw.pinPointOdo;

        // TODO: Tune these offsets for accurate positioning
        // odo.setOffsets(0, 865);
        pinpointDriver.setOffsets(120, -48);

        // Set the encoder resolution to the 4-bar pod type
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions to FORWARD for both encoders
        // This means x increases when moving forward, y increases when strafing left
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Initialize and reset control loop timer
        controllerLoopTime = new ElapsedTime();

        // Reset odometry position and IMU heading
        pinpointDriver.resetPosAndIMU();

        // Reset timer to start counting from zero
        controllerLoopTime.reset();
    }

    /**
     * Returns the number of times NaN readings were detected from the sensors.
     * @return count of NaN occurrences
     */
    public int getNanCounter(){
        return nanCounter;
    }

    /**
     * Returns the time elapsed since the last control loop iteration in seconds.
     * @return elapsed time in seconds
     */
    public double getControlLoopTime(){
        return controllerLoopTime.seconds();
    }

    /**
     * Updates odometry measurements from the GoBildaPinpointDriver.
     * Converts encoder positions to cm by dividing by 10.
     * Adjusts y-axis sign to fit coordinate convention (positive y when strafing right).
     */
    public void processOdometry(){
        x =  (pinpointDriver.getPosX() / 10);           // Convert mm or encoder units to cm for x
        y = -(pinpointDriver.getPosY() / 10);           // Convert and invert y to match coordinate system
        heading = pinpointDriver.getHeading();          // Get current heading in degrees
        pinpointDriver.update();                        // Update internal odometry data
    }

    /**
     * Getter for velocity in x direction.
     * @return velocity in x (units consistent with odometry)
     */
    public double getVx(){
        return vx;
    }

    /**
     * Getter for velocity in y direction.
     * @return velocity in y
     */
    public double getVy(){
        return vy;
    }

    /**
     * Getter for angular velocity (heading rate).
     * @return angular velocity (degrees per time unit)
     */
    public double getVtheta(){
        return vtheta;
    }

    /**
     * Dead reckoning update method.
     * If odometry readings are NaN (invalid), it estimates new position based on previous pose and velocities.
     * Otherwise, it updates pose from sensor data and velocity readings.
     */
    public void deadReckoning(){
        pinpointDriver.update();                       // Update odometry sensor

        // Read current raw position and heading from sensor
        Double checkX = pinpointDriver.getPosX();
        Double checkY = pinpointDriver.getPosY();
        Double checkHeading = pinpointDriver.getHeading();

        // Check if any reading is NaN (invalid)
        if (checkX.isNaN() || checkY.isNaN() || checkHeading.isNaN()){
            nanCounter++;  // Increment NaN counter

            // Estimate new pose by adding displacement since last valid update
            // controllerLoopTime.milliseconds() used as time delta
            x = previousX + (vx / 10 * controllerLoopTime.milliseconds());
            y = previousY + (vy / 10 * controllerLoopTime.milliseconds());
            heading = previousHeading + (vtheta * controllerLoopTime.milliseconds());

            // Alternative fallback (commented out) - keep pose static on invalid reading
            // x = previousX;
            // y = previousY;
            // heading = previousHeading;
        } else {
            // If readings valid, update pose from sensor, with unit conversions and sign adjustments
            x =  (pinpointDriver.getPosX() / 10);
            y = -(pinpointDriver.getPosY() / 10);
            heading = pinpointDriver.getHeading();

            // Save current pose for next dead reckoning step if needed
            previousX = x;
            previousY = y;
            previousHeading = heading;

            // Update velocity readings from odometry
            vx = pinpointDriver.getVelX();
            vy = pinpointDriver.getVelY();
            vtheta = pinpointDriver.getHeadingVelocity();
        }

        // Reset timer for next control loop
        controllerLoopTime.reset();
    }

    /**
     * Set a new known position and heading on the odometry driver.
     * Useful for resetting or correcting pose estimate.
     * @param x new x position (in cm)
     * @param y new y position (in cm)
     * @param heading new heading (in degrees)
     */
    public void setNewPosition(double x, double y, double heading){
        pinpointDriver.setPosition(new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, heading));
    }

    /**
     * Get raw encoder count for x-axis odometry.
     * @return raw encoder value for x
     */
    public double getRawX(){
        return pinpointDriver.getEncoderX();
    }

    /**
     * Get raw encoder count for y-axis odometry.
     * @return raw encoder value for y
     */
    public double getRawY(){
        return pinpointDriver.getEncoderY();
    }

    /**
     * Reset odometry position and IMU heading.
     * Useful for initialization or re-zeroing.
     */
    public void reset() {
        pinpointDriver.resetPosAndIMU();
    }

    /**
     * Get current estimated x position in cm.
     * @return current x position
     */
    public double getX(){
        return x;
    }

    /**
     * Get current estimated y position in cm.
     * @return current y position
     */
    public double getY(){
        return y;
    }

    /**
     * Get current estimated heading in degrees. (through the Driver)
     * @return current heading
     */
    public double getHeading(){
        return pinpointDriver.getHeading();
    }
}
