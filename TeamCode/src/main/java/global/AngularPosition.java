package global;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AngularPosition {
    public ModernRoboticsI2cCompassSensor compassSensor;
    public BNO055IMU gyroSensor;
    public boolean calibratingCompass = true;

    public double lastAngle = 0;
    public double heading = 0;

    public void init(HardwareMap hwMap){
        compassSensor = hwMap.get(ModernRoboticsI2cCompassSensor.class, "cp");
        gyroSensor = hwMap.get(BNO055IMU.class, "gyro");

        compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);

        initGyro();
        resetGyro();

    }

    public double getHeadingCS(){
        return compassSensor.getDirection();
    }

    public void setCompassMode () {
        if (calibratingCompass && !compassSensor.isCalibrating()) {
            compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
            calibratingCompass = false;
        }
    }
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyroSensor.initialize(parameters);
    }

    public void resetGyro() {
        lastAngle = (int) gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        heading = 0;
    }
    public double getHeadingGY() {
        double ca = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double da = ca - lastAngle;
        if (da < -180)
            da += 360;
        else if (da > 180)
            da -= 360;
        heading += da;
        lastAngle = ca;
        return heading;
    }

}
