package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Swerve.DiffySwerve.Aarush;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveDrive {
    public double
            rotationalPower, moduleAngle180, angleToTarget, angleFromTarget180, topModuleVelocity, bottomModuleVelocity;
    public double
            moduleAngle = 0, currentAngleTicks = 0,
            tuner = 2,
            ANGLE_MARGIN_OF_ERROR = Math.toRadians(15),
            bigGearRatio = 23 / 68,
            ticksPerRev = 145.1,
            rotationsPerSec = 1150 / 60,
            MAX_RPS_TICKS = rotationsPerSec * ticksPerRev, // 2781
            ticksPerRotation = 432;
//            TICKS_PER_DEGREE_BIG_GEAR = MAX_RPS_TICKS * bigGearRatio;

    public boolean isDisabled = false;


    private int
            topPosition = 0, bottomPosition = 0,
            topRotationPosition = 0,
            bottomRotationPosition = 0,
            topOffsetDirectional = 0,
            bottomOffsetDirectional = 0;

    public DcMotorEx
            topModule, bottomModule;

    // How i calculated: well, the MAX angle that you could ever reach is 90
    // since you have two pivoting points (if you have one turning point, to get to any point from either direction,
    // that is 180 degrees. two pivot points, that's max 90.
    // since you're also moving, i devoted 0.6 out of 1 whole bit of power to turning.
    // that is 0.6 (total power) / 90 (max angle) to get 0.0067, or applying 0.6 power if the angle is 90. less if less!
    public SwervePID powerController = new SwervePID(0.1, 0, 0);

    Telemetry telemetry;

    public SwerveDrive(Telemetry telemetry, DcMotorEx top, DcMotorEx bottom) {
        this.telemetry = telemetry;
        topModule = top;
        bottomModule = bottom;
    }

    public void updateEncoderPositions() {
        // Get encoder positions in degrees
        topPosition = topModule.getCurrentPosition();
        bottomPosition = bottomModule.getCurrentPosition();

        // Print encoder values for debugging
        telemetry.addData("Top Encoder Position", topPosition);
        telemetry.addData("Bottom Encoder Position", bottomPosition);
}

    public double getAngle() {
        currentAngleTicks = (topPosition + bottomPosition) / 2.0;
        moduleAngle = angleWrapNative((currentAngleTicks / ticksPerRotation) * 360);
        return moduleAngle;
    }

    public double angleWrapLoop(double angle) {
        while (Math.abs(angle) > 360) {
            if (Math.signum(angle) == -1)
                angle = angle + 360;
            else {
                angle = angle - 360;
            }
        }
        return angle;
    }

    public static double smallestAngleBetween(double angle1, double angle2) {
        // Ensure the angles are within 0-360 range
        angle1 = angle1 % 360;
        angle2 = angle2 % 360;

        // Compute the difference
        double diff = Math.abs(angle1 - angle2);
        return Math.min(diff, 360 - diff);
    }

    public double angleWrapNative(double angle) {
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public void moduleController(double velocity, double targetAngle, double powerMultiplierForMaxSpeed, double actualAngle) {

//        targetAngle = angleWrap(targetAngle-90);
        telemetry.addLine("--- Module.java ---");

        this.updateEncoderPositions();


        // Get both reachable angles for the module itself in deg.
        moduleAngle = angleWrap(actualAngle);
        moduleAngle180 = angleWrap(moduleAngle + Math.toRadians(180));

        angleToTarget = angleWrap(targetAngle - moduleAngle);
        angleFromTarget180 = angleWrap(targetAngle - moduleAngle180);

        if (Math.abs(angleToTarget) > Math.abs(angleFromTarget180)) {
            // Set the rotational power based on the PID for the angle
            rotationalPower = powerController.runPID(moduleAngle,angleFromTarget180);
        }
        else {
            // Set the rotational power based on the PID for the angle
            rotationalPower = powerController.runPID(moduleAngle, angleToTarget);
        }

//        if (Math.abs(angleFromTarget180) < ANGLE_MARGIN_OF_ERROR) {
//            velocity = 0;
//        }
//        else if (Math.abs(angleToTarget) < ANGLE_MARGIN_OF_ERROR) {
//            velocity = 0;
//        }

        if (Math.abs(angleFromTarget180) < ANGLE_MARGIN_OF_ERROR) {
            velocity = -velocity;
        }
        else if (Math.abs(angleToTarget) > ANGLE_MARGIN_OF_ERROR) {
            velocity = 0;
        }

        double topModulePower = (velocity * powerMultiplierForMaxSpeed) + rotationalPower;
        double bottomModulePower = (-velocity * powerMultiplierForMaxSpeed) + rotationalPower;

        // Gets the max magnitude, if it is greater that one.
        double magnitude = Math.max(
                1,
                Math.max(
                        topModulePower,
                        bottomModulePower
                )
        );

        if (!isDisabled) {
            topModule.setPower(
                    topModulePower / magnitude
            );
            bottomModule.setPower(
                    bottomModulePower / magnitude
            );
        }



//        telemetry.addData("angle:", getAngle());
        telemetry.addLine(String.format("power: %f %f", topModulePower / magnitude, bottomModulePower));
        telemetry.addLine(String.format("target, and module angles: %f %f %f", Math.toDegrees(targetAngle), Math.toDegrees(moduleAngle), Math.toDegrees(moduleAngle180)));
        telemetry.addLine(String.format("angles to: %f %f", Math.toDegrees(angleToTarget), Math.toDegrees(angleFromTarget180)));

        telemetry.addData("rPower:", rotationalPower);
//        telemetry.addData("velocity:", velocity);
    }
}