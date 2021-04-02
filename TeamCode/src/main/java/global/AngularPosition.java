package global;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import globalfunctions.Constants;

public class AngularPosition {
    public ModernRoboticsI2cCompassSensor compassSensor;
    public BNO055IMU leftGyro;
    public BNO055IMU rightGyro;
//    public boolean calibratingCompass = true;

//    public double lastHeadingGY = 0;
//    public double headingGY = 0;

    public double addLeftGY = 0;
    public double addRightGY = 0;

    public boolean isFailing = false;


    public void init(HardwareMap hwMap){
        compassSensor = hwMap.get(ModernRoboticsI2cCompassSensor.class, "cp");
        leftGyro = hwMap.get(BNO055IMU.class, "gyrol");
        rightGyro = hwMap.get(BNO055IMU.class, "gyror");

        compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);

        initGyro();
        resetGyro();

    }

    public double getHeading(double robotTheta) {
        double headingGY = getHeadingGY();
        double headingCS = getHeadingCS();
        boolean gyAccurate = Math.abs(robotTheta - headingGY) < Constants.ANGLE_ACCURACY;
        boolean csAccurate = Math.abs(robotTheta - headingCS) < Constants.ANGLE_ACCURACY;
        isFailing = !gyAccurate && !csAccurate;
        if (gyAccurate && csAccurate) {
            return 0.5 * (headingGY + headingCS);
        } else if (gyAccurate) {
            return headingGY;
        } else if (csAccurate) {
            return headingCS;
        }
        return 0;
    }

    public double getHeading() {
        return 0.5 * (getHeadingGY() + getHeadingCS());
    }

    public double getHeadingCS(){
        double dir = Constants.COMPASS_START - compassSensor.getDirection();
        if (dir < 0) {
            dir += 360;
        }
        return dir;
    }

//    public void setCompassMode () {
//        if (calibratingCompass && !compassSensor.isCalibrating()) {
//            compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
//            calibratingCompass = false;
//        }
//    }
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        leftGyro.initialize(parameters);
        rightGyro.initialize(parameters);
    }

    public void resetGyro() {
//        addGY = (int) gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        headingGY = 0;
        addLeftGY = getHeadingCS() - getAngle(leftGyro);
        addRightGY = getHeadingCS() - getAngle(rightGyro);
    }
    public float getAngle(BNO055IMU gyro) {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public double getHeadingLeftGY() {
        double ang = getAngle(leftGyro) + addLeftGY;

        if (ang < -180) {
            ang += 360;
        } else if (ang > 180) {
            ang -= 360;
        }
        return ang;
    }
    public double getHeadingRightGY() {
        double ang = getAngle(rightGyro) + addRightGY;

        if (ang < -180) {
            ang += 360;
        } else if (ang > 180) {
            ang -= 360;
        }

        return ang;
    }
    public double getHeadingGY() {
//        double ang = 0.5 * (getAngle(leftGyro) + getAngle(rightGyro)) + addGY;
//
//        ang = ang < 0 ? (ang + 360) : ang;
        double ang = 0.5 * (getHeadingLeftGY() + getHeadingRightGY());
        if (ang < 0) {
            ang += 360;
        }
        return ang;
    }

}
