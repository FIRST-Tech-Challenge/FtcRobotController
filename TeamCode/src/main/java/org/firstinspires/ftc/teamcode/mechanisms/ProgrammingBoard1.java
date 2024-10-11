package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

/**
 * A Class that contains hardware and basic operational functions for all
 * the hardware in the programming board derived from "Learn Java for FTC"
 * by Alan Smith
 */
public class ProgrammingBoard1 {
    private DigitalChannel touchSensor;

    private Servo servo;

    boolean ch8ex1 = false;

    private AnalogInput pot;

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private DistanceSensor distanceSensor2m;
    private IMU imu;

    boolean bDistanceColor = true;
    boolean bDistance2m = true;

    /**
     * Primary initalization and some setup of all hardware devices on the
     * programming board
     *
     * @param hwMap
     */
    public void init(HardwareMap hwMap) {
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        servo = hwMap.get(Servo.class, "servo");
        // set true to see ch8 exercise 1
        if (ch8ex1) {
            servo.setDirection(Servo.Direction.REVERSE);
            servo.scaleRange(0.0, 0.5);
        }
        pot = hwMap.get(AnalogInput.class, "pot");

        if (bDistanceColor) {
            colorSensor = hwMap.get(ColorSensor.class, "sensor_color_distance");
            distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");
        }

        if (bDistance2m) {
            distanceSensor2m = hwMap.get(DistanceSensor.class, "distSense2m");
        }

        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.
                        LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revOrientation));
    }

    public boolean isTouchSensorPressed() {
        return !touchSensor.getState();
    }


    public void setServoPosition(double position) {
        servo.setPosition(position);
    }

    boolean ch9ex1 = false;
    boolean ch9ex2 = false;

    public boolean getCh9Ex2() {
        return ch9ex2;
    }

    public double getPotAngle() {
        if (ch9ex1 || ch9ex2) {
            return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 1);
        } else {
            return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 270);
        }
    }

    public int getAmountRed() {
        return colorSensor.red();
    }

    public int getAmountBlue() {
        return colorSensor.blue();
    }

    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }

    public double getDist2m(DistanceUnit du) {
        return distanceSensor2m.getDistance(du);
    }

    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * Primary selection array of test for the test OpMode
     *
     * @return
     */
    public ArrayList<TestItem> getTests() {
        ArrayList<TestItem> tests = new ArrayList<>();
        tests.add(new TestAnalogInput("PB Pot", pot, 0, 270));
        tests.add(new TestDigitalChannel("PB TouchSensor", touchSensor));
        tests.add(new TestServo("PB Servo", servo, 0.0, 1.0));
        tests.add(new TestColorDistanceSensor("PB ColorDistanceSensor", colorSensor, distanceSensor));
        return tests;
    }

}
