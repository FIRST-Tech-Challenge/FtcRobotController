package org.firstinspires.ftc.forteaching.TechnoBot;

// This is a sample hardware class that keeps a reference to all the
// pieces of hardware on the robot

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.hardware.sensor.Rev2MDistanceSensor;
import com.technototes.library.hardware.servo.Servo;

public class Hardware {
    // Put all the names of the devices in here:
    @Config
    public static class DeviceNames {
        public static String LEFT_MOTOR = "motorL";
        public static String RIGHT_MOTOR = "motorR";
        public static String IMU = "imu";
        public static String REV2M_DIST = "distance";
        public static String SERVO = "servo";
        public static String BUMP = "bump";
        public static String COLOR = "color";
    }

    // We make this public so subsystems & whatnot can get them
    public Motor<DcMotorEx> leftDriveMotor;
    public Motor<DcMotorEx> rightDriveMotor;
    public IMU inertialMovementUnit;
    public Rev2MDistanceSensor distanceSensor;
    public Servo spinnerServo;
    public RevTouchSensor bumpSensor;
    public RevColorSensorV3 colorSensor;

    public Hardware(HardwareMap hwmap) {
        if (TheBot.Connected.DriveTrain) {
            leftDriveMotor = new Motor<DcMotorEx>(DeviceNames.LEFT_MOTOR);
            rightDriveMotor = new Motor<DcMotorEx>(DeviceNames.RIGHT_MOTOR);
            inertialMovementUnit = new IMU(DeviceNames.IMU);
        }
        if (TheBot.Connected.Sensors) {
            distanceSensor = new Rev2MDistanceSensor(DeviceNames.REV2M_DIST);
            spinnerServo = new Servo(DeviceNames.SERVO);
            bumpSensor = hwmap.get(RevTouchSensor.class, DeviceNames.BUMP);
            colorSensor = hwmap.get(RevColorSensorV3.class, DeviceNames.COLOR);
        }
    }
}
