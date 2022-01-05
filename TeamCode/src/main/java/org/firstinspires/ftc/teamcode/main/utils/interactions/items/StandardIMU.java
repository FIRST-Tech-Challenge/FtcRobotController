package org.firstinspires.ftc.teamcode.main.utils.interactions.items;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;

import java.util.Hashtable;

public class StandardIMU extends InteractionItem {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    float headingOffset;

    public float getHeadingOffset() {
        return headingOffset;
    }
    public void setHeadingOffset(float offset) {
        this.headingOffset = offset;
    }

    float rollOffset;
    public float getRollOffset() {
        return rollOffset;
    }
    public void setRollOffset(float offset) {
        this.rollOffset = offset;
    }

    float pitchOffset;
    public float getPitchOffset() {
        return headingOffset;
    }
    public void setPitchOffset(float offset) {
        this.pitchOffset = offset;
    }

    // State used for updating telemetry
    Orientation angles;

    public StandardIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public ReturnData<DataPoint, Float> getData() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float heading = -angles.firstAngle + headingOffset;
        /*if (heading < 0) {
            heading = 360 + heading;
        }*/

        float roll = angles.secondAngle + rollOffset;
        /*if (roll < 0) {
            roll = 360 + roll;
        }*/

        float pitch = angles.secondAngle + pitchOffset;
        /*if (pitch < 0) {
            pitch = 360 + pitch;
        }*/

        ReturnData<DataPoint, Float> toReturn = new ReturnData<>();
        toReturn.put(DataPoint.HEADING, heading);
        toReturn.put(DataPoint.ROLL, roll);
        toReturn.put(DataPoint.PITCH, pitch);

        return toReturn;
    }

    @Override
    public void stop() {}

    @Override
    public boolean isInputDevice() {
        return false;
    }

    @Override
    public boolean isOutputDevice() {
        return true;
    }

    public enum DataPoint {HEADING, PITCH, ROLL}

    public static class ReturnData<K, V> extends Hashtable<K,V> {
        public ReturnData() {
            super();
        }

        public float getHeading() {
            return (float) this.get(DataPoint.HEADING);
        }

        public float getPitch() {
            return (float) this.get(DataPoint.PITCH);
        }

        public float getRoll() {
            return (float) this.get(DataPoint.ROLL);
        }
    }
}