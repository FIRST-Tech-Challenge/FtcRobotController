package org.firstinspires.ftc.teamcode.ebotssensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

public class EbotsImu {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private static EbotsImu ebotsImu = null;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private BNO055IMU imuSensor;
    BNO055IMU.Parameters parameters;
    private double fieldHeadingWhenInitializedDeg = 0;
    private double currentFieldHeading = 0;
    private StopWatch stopWatchReading = new StopWatch();   // time since last hardware read
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private EbotsImu(HardwareMap hardwareMap){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        initEbotsImu(hardwareMap);


    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    public double getFieldHeadingWhenInitializedDeg() {
        double value = (ebotsImu != null) ? ebotsImu.fieldHeadingWhenInitializedDeg : 0;
        return value;
    }

    public void setFieldHeadingWhenInitializedDeg(double fieldHeadingWhenInitializedDeg) {
        if (ebotsImu != null) {
            ebotsImu.fieldHeadingWhenInitializedDeg = fieldHeadingWhenInitializedDeg;
        }
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public static EbotsImu getInstance(HardwareMap hardwareMap){

        if (ebotsImu == null){
            ebotsImu = new EbotsImu(hardwareMap);
        }
        return ebotsImu;
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public void initEbotsImu(HardwareMap hardwareMap){
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imuSensor = hardwareMap.get(BNO055IMU.class, "imu");
        imuSensor.initialize(parameters);

    }

    public double performHardwareRead(){
        double readingDeg = 0;
        if (ebotsImu != null) {
            ebotsImu.stopWatchReading.reset();  // reset timer that keeps track of time since last hardware read
            readingDeg = ebotsImu.imuSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
        return readingDeg;
    }

    private void updateFieldHeading(){
        if (ebotsImu != null) {
            double imuReading = ebotsImu.performHardwareRead();
            ebotsImu.currentFieldHeading = UtilFuncs.applyAngleBounds(imuReading + ebotsImu.fieldHeadingWhenInitializedDeg);
        }
    }
    /**
     * getCurrentHeadingDeg() is a time-buffered value cache for the hardware reading
     * if a hardware read is necessary, call updateHeading first
     * @return
     * @param forceHardwareRead
     */

    public double getCurrentFieldHeadingDeg(boolean forceHardwareRead) {
        if (ebotsImu == null) return 0;

        long headingRefreshRateMillis = 500;
        boolean timeBufferExpired = ebotsImu.stopWatchReading.getElapsedTimeMillis() > headingRefreshRateMillis;

        if (timeBufferExpired | forceHardwareRead){
            updateFieldHeading();
        }
        return ebotsImu.currentFieldHeading;
    }



}
