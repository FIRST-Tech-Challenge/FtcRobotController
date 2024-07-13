package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Dropper extends Component {
    private final ColorRangeSensor lowerSensor;
    public static double lowerSensorThreshold = 15.0;
    private final ColorRangeSensor upperSensor;
    public static double upperSensorThreshold = 15.0;
    private final Servo lowerServo;
    private final Servo upperServo;
    private final double upperServoLockPos = 1;
    private final double upperServoUnlockPos = 0.5;
    private final double lowerServoLockPos = 1;
    private final double lowerServoUnlockPos = 0.5;

    private boolean lowerLocked = false;
    private boolean upperLocked = false;
    private int loadSize;

    public Dropper(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn) {
        super(telemetry, loggingOn);
        lowerSensor = hardwareMap.get(ColorRangeSensor.class, "dropperLowerSensor");
        upperSensor = hardwareMap.get(ColorRangeSensor.class, "dropperUpperSensor");
        lowerServo = hardwareMap.get(Servo.class, "dropperLowerServo");
        upperServo = hardwareMap.get(Servo.class, "dropperUpperServo");
        lockServos();
    }

    private void log()
    {
        telemetry.addData("upperSensor: ", upperSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("lowerSensor: ", lowerSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("loadSize: ", loadSize);
        telemetry.update();
    }
    public void dropPixel() {
        if (lowerLocked) {
            unlockLowerServo();
        } else {
            unlockUpperServo();
        }
    }

    public void load() {
        unlockServos();
    }

    private int countPixels() {
        int count = 0;
        if (upperPixelPresent()) {
            count++;
        }
        if (lowerPixelPresent()) {
            count++;
        }
        return (count);
    }

    public void stopLoad() {
        lockServos();
    }

    private boolean upperPixelPresent() {
        return (upperSensor.getDistance(DistanceUnit.MM) < upperSensorThreshold);
    }

    private boolean lowerPixelPresent() {
        return (lowerSensor.getDistance(DistanceUnit.MM) < lowerSensorThreshold);
    }

    public boolean fullyLoaded() {
        return (loadSize == 2);
    }

    private void lockUpperServo() {
        upperServo.setPosition(upperServoLockPos);
        upperLocked = true;
    }

    private void lockLowerServo() {
        lowerServo.setPosition(lowerServoLockPos);
        lowerLocked = true;
    }

    private void unlockUpperServo() {
        upperServo.setPosition(upperServoUnlockPos);
        upperLocked = false;
    }

    private void unlockLowerServo() {
        lowerServo.setPosition(lowerServoUnlockPos);
        lowerLocked = false;
    }

    public void lockServos() {
        lockUpperServo();
        lockLowerServo();
    }

    public void unlockServos() {
        unlockUpperServo();
        unlockLowerServo();
    }

    public void update() {
        loadSize = countPixels();
        if (loggingOn)
        {
            log();
        }
    }
}
