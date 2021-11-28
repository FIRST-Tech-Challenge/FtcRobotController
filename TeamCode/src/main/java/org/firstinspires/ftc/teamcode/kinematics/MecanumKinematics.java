package org.firstinspires.ftc.teamcode.kinematics;

import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

import org.firstinspires.ftc.teamcode.drive.MotorDeltaReport;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

public class MecanumKinematics {

    private static MecanumKinematics instance = new MecanumKinematics();

    private MecanumKinematics() { }

    public static MecanumKinematics getInstance() {
        return instance;
    }

    public ChassisCommand toWheelSpeeds(Pose2D cmd) {
        double lb = HALF_TRACK_WIDTH + HALF_WHEEL_BASE;
        return new ChassisCommand(cmd.y - cmd.x - cmd.theta*lb,
                                  cmd.y + cmd.x + cmd.theta*lb,
                                  cmd.y + cmd.x - cmd.theta*lb,
                                  cmd.y - cmd.x + cmd.theta*lb).scale(1/WHEEL_CIRCUMFERENCE);
    }

    public Vector2D toRobotSpeed(MotorDeltaReport d) {
        return new Vector2D(d.lf + d.rf + d.lr + d.rr, -d.lf + d.rf + d.lr - d.rr).scale(WHEEL_CIRCUMFERENCE / 4);
    }

    public ChassisCommand fromGamepad(double x, double y, double w) {
        Vector2D translation = new Vector2D(x, y).scaleExp(GAMEPAD_DRIVE_SMOOTHER).scale(MAX_DRIVE_SPEED);
        return toWheelSpeeds(new Pose2D(translation, MAX_TURN_SPEED * Math.pow(w, GAMEPAD_TURN_SMOOTHER)));
    }

    public Pose2D localToGlobalMovement(double heading, Vector2D translation, double dHeading) {
        // Integrate movement locally
        Vector2D dT;
        if (dHeading != 0) {
            dT = new Vector2D((Math.sin(dHeading) / dHeading) * translation.x + - (1-Math.cos(dHeading)/dHeading) * translation.y,
                              (1-Math.cos(dHeading)/dHeading) * translation.x + (Math.sin(dHeading) / dHeading) * translation.y);
        } else {
            dT = translation.copy();
        }

        // Rotate the movement
        return new Pose2D(Math.cos(heading)*dT.x - Math.sin(heading)*dT.y,
                          Math.sin(heading)*dT.x + Math.cos(heading)*dT.y,
                            dHeading);
    }
}
