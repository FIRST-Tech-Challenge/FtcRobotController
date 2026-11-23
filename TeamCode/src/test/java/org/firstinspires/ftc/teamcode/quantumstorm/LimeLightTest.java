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
        servo.updatePickup();
        check(position[0] == .9 && servo.getActivePipeline() == LimeServo.Pipeline.YELLOW,
                "pickup begins yellow at the color angle");
        now[0] = 9;
        servo.updatePickup();
        check(position[0] == .9 && servo.getActivePipeline() == LimeServo.Pipeline.BLUE,
                "pickup checks blue without changing tilt");
        now[0] = 12;
        servo.updatePickup();
        check(servo.getActivePipeline() == LimeServo.Pipeline.RED, "pickup checks red for avoidance");
        now[0] = 15;
        servo.updatePickup();
        check(servo.getActivePipeline() == LimeServo.Pipeline.YELLOW, "pickup cycle returns to yellow");
        servo.update(true);
        check(position[0] == .5, "shooting interrupts pickup and restores the starting neutral");
        servo.selectPipeline(LimeServo.Pipeline.YELLOW);
        check(position[0] == .9, "parking color pose stays 120 degrees clockwise without scanning");
        now[0] = 20;
        servo.update(false);
        now[0] = 22.999;
        servo.update(false);
        check(servo.getActivePipeline() == LimeServo.Pipeline.RED, "scan restarts with a full dwell");
        now[0] = 0;
        servo.resetTiming();
        now[0] = 2.999;
        servo.update(false);
        check(servo.getActivePipeline() == LimeServo.Pipeline.RED,
                "clock reset preserves the new three-second dwell");
        now[0] = 3;
        servo.update(false);
        check(servo.getActivePipeline() == LimeServo.Pipeline.BLUE,
                "post-START timing does not wait for the pre-START clock to catch up");
        System.out.println("Neutral, shooting, parking tilt and idle/pickup scan timing passed");
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
}
