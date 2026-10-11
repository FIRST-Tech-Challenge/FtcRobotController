package org.firstinspires.ftc.teamcode.maths;

public class swerveKinematics {

    public double[] calculate(double forward, double strafe, double rotate, double imu,
                              boolean fieldcentric, double radius) {

        // FIX: Removed "fieldcentric = true;" here so the user can actually toggle it.

        // 1. Field Centric Adjustment
        double strafe1 = strafe;
        double forward1 = forward;

        if (fieldcentric) {
            // Optimization: Convert once
            double rad = Math.toRadians(imu);
            double sin = Math.sin(rad);
            double cos = Math.cos(rad);

            strafe1 = cos * strafe - sin * forward;
            forward1 = sin * strafe + cos * forward;
        }

        // 2. Kinematics (Wheel Specific Vectors)
        // Vx = strafe - rot * Ry
        // Vy = forward + rot * Rx

        // Module 1: Right (X=1, Y=-0)
        double mod1strafe = strafe1 - (rotate * radius * -0.0);
        double mod1forward = forward1 + (rotate * radius * 1.0);

        // Module 2: Left (X=-1, Y=-0)
        double mod2strafe = strafe1 - (rotate * radius * -0.0);
        double mod2forward = forward1 + (rotate * radius * -1.0);

        // 3. Extract Speed (Magnitude)
        double mod1speed = Math.sqrt((mod1strafe * mod1strafe) + (mod1forward * mod1forward));
        double mod2speed = Math.sqrt((mod2strafe * mod2strafe) + (mod2forward * mod2forward));

        // 4. Normalize Speeds (Don't exceed 1.0)
        double max1 = Math.max(Math.abs(mod2speed), Math.abs(mod1speed));
        if (Math.abs(max1) > 1) {
            mod1speed /= Math.abs(max1);
            mod2speed /= Math.abs(max1);
        }

        // 5. Extract Angle (Atan2)
        double mod1angle = Math.atan2(mod1strafe, mod1forward) * 180 / Math.PI;
        double mod2angle = Math.atan2(mod2strafe, mod2forward) * 180 / Math.PI;

        return new double[]{mod1speed, mod2speed, mod1angle, mod2angle};
    }
}
