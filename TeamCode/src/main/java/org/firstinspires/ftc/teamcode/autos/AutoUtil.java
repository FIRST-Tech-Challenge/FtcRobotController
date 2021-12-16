package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AutoUtil {
    static ElapsedTime runtime = new ElapsedTime();
    public static void delay(int time, SampleMecanumDrive drive, LinearOpMode opMode) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time && !opMode.isStopRequested()) {
            drive.update();
        }
    }

    public static void delay(int time, LinearOpMode opMode) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time && !opMode.isStopRequested()) {
            // do nothing
        }
    }

    // This is copied from stack overflow
    public static <T> T mostCommon(List<T> list) {
        Map<T, Integer> map = new HashMap<>();

        for (T t : list) {
            Integer val = map.get(t);
            map.put(t, val == null ? 1 : val + 1);
        }

        Map.Entry<T, Integer> max = null;

        for (Map.Entry<T, Integer> e : map.entrySet()) {
            if (max == null || e.getValue() > max.getValue())
                max = e;
        }

        return max.getKey();
    }
}
