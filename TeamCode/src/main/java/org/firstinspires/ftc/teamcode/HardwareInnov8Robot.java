package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class HardwareInnov8Robot {
    /* Public OpMode members. */
    public DcMotor motorOne = null; // Front left wheel
    public DcMotor motorTwo = null; // Back left  wheel
    public DcMotor motorThree = null; // Front right wheel
    public DcMotor motorFour = null; // Back right wheel


    // Example for servos

    //    public Servo


    public BNO055IMU imu;
//
    // Examples for servos

    public static final double MID_SERVO = 0.5;
    public static final double START_SERVO = 0; // all the way down
    public static final double END_SERVO = 1; // all the way up

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareInnov8Robot(HardwareMap ahwMap) {
        this.hwMap = ahwMap;
        this.init(ahwMap);

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map


        // Define and Initialize Motors
        motorOne = this.hwMap.get(DcMotor.class, "motorOne");
        motorTwo = this.hwMap.get(DcMotor.class, "motorTwo");
        motorThree = this.hwMap.get(DcMotor.class, "motorThree");
        motorFour = this.hwMap.get(DcMotor.class, "motorFour");
        motorOne.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorTwo.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorThree.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorFour.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Set all motors to zero power
        motorOne.setPower(0);
        motorTwo.setPower(0);
        motorThree.setPower(0);
        motorFour.setPower(0);



        // Set all motors to run without encoders.

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFour.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Define and initialize ALL installed servos.



    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
