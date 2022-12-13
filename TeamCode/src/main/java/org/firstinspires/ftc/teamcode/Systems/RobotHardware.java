package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;

public class RobotHardware implements Robot {

    public BNO055IMU imu;

    public HardwareMap map;
    public Telemetry telemetry;

    public DcMotor[] wheel_list = new DcMotor[wheel_names.size()];
    public DcMotor[] dc_motor_list = new DcMotor[dc_motor_names.size()];
    public Servo[] servo_list = new Servo[servo_names.size()];
    public CRServo[] cr_servo_list = new CRServo[cr_servo_names.size()];

    public DistanceSensor[] distance_sensor_list = new DistanceSensor[distance_sensor_names.size()];
    public TouchSensor[] touch_sensor_list = new TouchSensor[touch_sensor_names.size()];
    public ColorSensor[] color_sensor_list = new ColorSensor[color_sensor_names.size()];
    public RevBlinkinLedDriver[] led_list = new RevBlinkinLedDriver[led_names.size()];
    
    double imu_zero;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu_zero = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS).firstAngle;

        for (int i = 0; i < wheel_list.length; i++) {
            wheel_list[i] = hardwareMap.get(DcMotor.class, wheel_names.get(i));
            wheel_list[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheel_list[i].setDirection(i > 1 ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        }

        for (int i = 0; i < distance_sensor_list.length; i++)
            distance_sensor_list[i] = hardwareMap.get(DistanceSensor.class, distance_sensor_names.get(i));

        for (int i = 0; i < touch_sensor_list.length; i++)
            touch_sensor_list[i] = hardwareMap.get(TouchSensor.class, touch_sensor_names.get(i));
    
        for (int i = 0; i < color_sensor_list.length; i++)
            color_sensor_list[i] = hardwareMap.get(ColorSensor.class, color_sensor_names.get(i));

        for (int i = 0; i < led_list.length; i++)
            led_list[i] = hardwareMap.get(RevBlinkinLedDriver.class, led_names.get(i));

        for (int i = 0; i < dc_motor_list.length; i++) {
            dc_motor_list[i] = hardwareMap.get(DcMotor.class, dc_motor_names.get(i));
            dc_motor_list[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dc_motor_list[i].setDirection(invert_dc_motors[i] ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            dc_motor_list[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dc_motor_list[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (int i = 0; i < servo_list.length; i++)
            servo_list[i] = hardwareMap.get(Servo.class, servo_names.get(i));

        for (int i = 0; i < cr_servo_list.length; i++) {
            cr_servo_list[i] = hardwareMap.get(CRServo.class, cr_servo_names.get(i));
            dc_motor_list[i].setDirection(invert_cr_servos[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        telemetry.addData("Robot Hardware", "Initialized");
        telemetry.update();

        this.map = hardwareMap;
    }

    //IMU Stuff
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS);
        return (angles.firstAngle - imu_zero) * (invertIMU ? -1 : 1);
    }

    public void turnDegree(double degrees) {
        double startingAngle = getAngle();

        double targetHeading = startingAngle + degrees;
        while (targetHeading < 0 || targetHeading >= 360) {
            targetHeading += 360 * (targetHeading < 0 ? 1 : -1);
        }

        int factor = (((0 < targetHeading - startingAngle) && (targetHeading - startingAngle < 180)) || (targetHeading - startingAngle < -180) ? 1 : -1);
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower((i < 2 ? -0.5 : 0.5) * factor);

        while ((getAngle() - targetHeading) * (getAngle() - targetHeading) < 25) {
            //wait
        }
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower(0);
    }

    //Voltage
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : map.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    //DC Motors
    public void setPower(String name, double power) {
        dc_motor_list[dc_motor_names.indexOf(name)].setPower(power);
    }

    public void resetEncoder(String name) {
        dc_motor_list[dc_motor_names.indexOf(name)].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dc_motor_list[dc_motor_names.indexOf(name)].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Servos
    public void setPosition(String name, double position) {
        servo_list[servo_names.indexOf(name)].setPosition(position);
    }

    //Distance Sensor
    public double getDistInch(String name){
        return distance_sensor_list[distance_sensor_names.indexOf(name)].getDistance(DistanceUnit.INCH);
    }

    //Touch Sensor
    public boolean touchSensorTouching(String name) {
        return touch_sensor_list[touch_sensor_names.indexOf(name)].isPressed();
    }

    //Color Sensor
    public int[] getRGBA(String name) {
        return new int[] {
            color_sensor_list[color_sensor_names.indexOf(name)].red(), 
            color_sensor_list[color_sensor_names.indexOf(name)].green(), 
            color_sensor_list[color_sensor_names.indexOf(name)].blue(), 
            color_sensor_list[color_sensor_names.indexOf(name)].alpha()
        };
    }

    //LED
    public void setLed(String name, String pattern) {
        RevBlinkinLedDriver.BlinkinPattern convertedPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        switch(pattern){
            case "Rainbow":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                break;
            case "Red":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                break;
            case "Orange":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                break;
            case "Yellow":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;
            case "Green":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
            case "Blue":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                break;
            case "Violet":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case "White":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                break;
            case "Black":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                break;
        }
        led_list[led_names.indexOf(name)].setPattern(convertedPattern);
    }

    public void stop() {
        for (DcMotor motor : dc_motor_list) {
            motor.setPower(0);
        }
        for (DcMotor motor : wheel_list) {
            motor.setPower(0);
        }
    }
}