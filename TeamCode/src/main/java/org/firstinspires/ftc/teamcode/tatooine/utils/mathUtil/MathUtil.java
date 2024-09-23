package org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil;

import com.acmerobotics.roadrunner.Vector2d;

public class MathUtil {
    public static Vector2d rotateVec(Vector2d vec, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Vector2d(

                vec.component1() * sinA + vec.component2() * cosA,
                vec.component1() * cosA - vec.component2() * sinA
        );
    }

    public static double aplayDeadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return input;
    }
}

