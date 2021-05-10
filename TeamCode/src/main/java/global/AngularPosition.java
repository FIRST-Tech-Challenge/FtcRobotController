package global;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import globalfunctions.Constants;
import globalfunctions.Optimizer;

public class AngularPosition {
    //Left gyro in expansion hub 1
    public BNO055IMU leftGyro;
    //Left gyro in expansion hub 2
    public BNO055IMU rightGyro;

    //Offset for left gyro
    public double addLeftGY = 0;
    //Offset for right gyro
    public double addRightGY = 0;
    //Is angular position failing?
    public boolean isFailing = false;
    //Number of checks failed
    public int checksFailed = 0;

    //Initialized gyro sensors and reset heading to 0
    public void init(HardwareMap hwMap){
        leftGyro = hwMap.get(BNO055IMU.class, "gyrol");
        rightGyro = hwMap.get(BNO055IMU.class, "gyror");
        initGyro();
        resetGyro(0);
    }
    //Get the heading if the old heading is close to the new one
    public double getHeading(double oldHeading) {
        double headingGY = getHeadingGY();
        boolean gyAccurate = Math.abs(oldHeading - headingGY) < Constants.ANGLE_ACCURACY;
        isFailing = !gyAccurate;
        if(isFailing){
            checksFailed +=1;
        }
        if (gyAccurate) {
            return headingGY;
        }else{
            return oldHeading;
        }
    }

    //Initialize gyro sensors
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        leftGyro.initialize(parameters);
        rightGyro.initialize(parameters);
    }
    //Reset the gyro sensors to a certain heading
    public void resetGyro(double heading) {
        addLeftGY = heading - getAngle(leftGyro);
        addRightGY = heading - getAngle(rightGyro);
    }
    //Get the angle from a gyro sensors
    public float getAngle(BNO055IMU gyro) {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    //Get the heading from the left gyro
    public double getHeadingLeftGY() {
        double ang = getAngle(leftGyro) + addLeftGY;

        if (ang < -180) {
            ang += 360;
        } else if (ang > 180) {
            ang -= 360;
        }
        return ang;
    }
    //Get the heading from the right gyro
    public double getHeadingRightGY() {
        double ang = getAngle(rightGyro) + addRightGY;

        if (ang < -180) {
            ang += 360;
        } else if (ang > 180) {
            ang -= 360;
        }

        return ang;
    }
    //Get the heading from both gyros without checking
    public double getHeadingGY() {
        double lgy = getHeadingLeftGY();
        double rgy = getHeadingRightGY();
        double ang = 0;
        if(Math.abs(lgy - rgy) < 20) {
             ang = Optimizer.weightedAvg(new double[]{lgy, rgy}, new double[]{1, 1});
        }else{
             ang = lgy;
        }
        if (ang < 0) {
            ang += 360;
        }

        return ang;
    }


}
