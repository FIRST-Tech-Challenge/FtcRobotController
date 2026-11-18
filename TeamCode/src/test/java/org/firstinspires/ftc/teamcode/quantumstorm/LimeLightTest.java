package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.quantumstorm.Auto.LimeServo;

public class LimeLightTest {
    public static void main(String[] args) {
        double[] now = {0};
        double[] position = {-1};
        Servo tilt = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (proxy, method, values) -> {
                    if (method.getName().equals("setPosition")) {
                        position[0] = (Double) values[0];
                        return null;
                    }
                    throw new UnsupportedOperationException(method.getName());
                });
        LimeServo servo = new LimeServo(null, tilt, () -> now[0]);
        check(position[0] == .5, "initial neutral position");
        servo.update(false);
        check(position[0] == .9 && servo.getActivePipeline() == LimeServo.Pipeline.RED,
                "idle starts red at 120 degrees clockwise");
        now[0] = 2.999;
        servo.update(false);
        check(servo.getActivePipeline() == LimeServo.Pipeline.RED, "no early pipeline switch");
        now[0] = 3;
        servo.update(false);
        check(servo.getActivePipeline() == LimeServo.Pipeline.BLUE, "switch at exactly three seconds");
        now[0] = 6;
        servo.update(false);
        check(servo.getActivePipeline() == LimeServo.Pipeline.RED, "red blue cycle repeats");
        servo.update(true);
        check(position[0] == .5 && servo.getActivePipeline() == LimeServo.Pipeline.APRIL_TAG,
                "shooting returns to neutral AprilTags");
        servo.selectPipeline(LimeServo.Pipeline.YELLOW);
        check(position[0] == .9, "yellow pickup keeps clockwise tilt");
        now[0] = 20;
        servo.update(false);
        now[0] = 22.999;
        servo.update(false);
        check(servo.getActivePipeline() == LimeServo.Pipeline.RED, "scan restarts with a full dwell");
        System.out.println("Neutral, tilt and red/blue scan timing passed");
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
}
