package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class InternalIMUSensor implements PositionChangeSensor {
    private BNO055IMU        imu;
    private Orientation   angles;
    private Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();
    private Position lastPosition;

    double lastT = 0;
    double lastHeading  = 0;

    private static final int pollInterval = 100;

    public InternalIMUSensor(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        lastPosition = new Position();

        imu.startAccelerationIntegration(new Position(), new Velocity(), pollInterval);
    }

    public void reset(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), pollInterval);
    }

    public void setPosition(Position position){
        imu.startAccelerationIntegration(position, null,pollInterval);
    }

    public Position getPosition(){
        return  imu.getPosition();
    }

    public float getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastHeading  = -angles.firstAngle;
        lastT     = runtime.seconds();

        return -  angles.firstAngle;
    }

    /** gets the Angle Change rate in degrees.
     *
     *
     * @return heading change rate in degrees
     */
    public double angleRate(){
        double lastH = lastHeading;
        double timeChange = runtime.seconds()-lastT;
        double heading  = getHeading();

        return (heading - lastH)/timeChange;
    }

    /** gets the Angle Change since you last asked in degrees.
     *
     *
     * @return heading change in degrees
     */
    public double angleChange() {
        double lastH = lastHeading;
        double heading = getHeading();

        return (heading - lastH);
    }

    @Override
    public double[] getStateChange() {
        double[] retVal = getStateChangeDegrees();
        retVal[2] *= Math.PI / 180.0;
        return retVal;
    }

    @Override
    public double[] getStateChangeDegrees() {
        Position thisPosition = imu.getPosition();

        double [] retVal = {thisPosition.x - lastPosition.x,
                thisPosition.y - lastPosition.y,
                angleChange() };
        lastPosition = thisPosition;
        return retVal;
    }
}
