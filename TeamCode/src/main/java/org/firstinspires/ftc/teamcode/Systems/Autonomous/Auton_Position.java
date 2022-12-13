package org.firstinspires.ftc.teamcode.Systems.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

public class Auton_Position implements Robot {

    StandardTrackingWheelLocalizer position_tracker;
    RobotHardware robot;
    double x; 
    double y;
    double angle; //ALL of these variables are based on field

    double current_time;
    double previous_time;

    double current_error;
    double previous_error;

    public Auton_Position(RobotHardware r, StandardTrackingWheelLocalizer l, double x_, double y_, double angle_) {
        position_tracker = l;
        robot = r;
        position_tracker.setPoseEstimate(new Pose2d(x, y, Math.toRadians(angle)));
        this.x = x_;
        this.y = y_;
        this.angle = Math.toRadians(angle_);
    }

    public void turn(double angle_) { //positive = right
        position_tracker.update();
        Pose2d current_pose = position_tracker.getPoseEstimate();
        angle = current_pose.getHeading();
        position_tracker.setPoseEstimate(new Pose2d(0, 0, 0));
        angle_ *= Math.PI / 180.0;
        current_error = 0;
        previous_error = 0;

        current_time = System.currentTimeMillis();
        previous_time = System.currentTimeMillis();

        if (angle_ > 0) {
            while (position_tracker.getPoseEstimate().getHeading() > 0 - angle_ + 0.1) {
                drive(5 * (position_tracker.getPoseEstimate().getHeading() + angle_), 0.4);
                position_tracker.update();
            }
        } else {
            while (position_tracker.getPoseEstimate().getY() < 0 - angle_ - 0.1) {
                drive(5 * (position_tracker.getPoseEstimate().getHeading() + angle_), 0.4);
                position_tracker.update();
            }
        }
        position_tracker.setPoseEstimate(new Pose2d(current_pose.getX(), current_pose.getY(), angle - angle_));
    }

    public void strafe(double inches) {
        //by default, positive = right
        position_tracker.update();
        Pose2d current_pose = position_tracker.getPoseEstimate();
        angle = current_pose.getHeading();
        position_tracker.setPoseEstimate(new Pose2d(0, 0, 0));

        current_error = 0;
        previous_error = 0;

        current_time = System.currentTimeMillis();
        previous_time = System.currentTimeMillis();

        if (inches > 0) {
            while (position_tracker.getPoseEstimate().getY() > 0 - inches + 0.1) {
                drive(0.6 * (position_tracker.getPoseEstimate().getY() + inches), Math.PI / 2.0, 0.4);
                position_tracker.update();
            }
        } else {
            while (position_tracker.getPoseEstimate().getY() < 0 - inches - 0.1) {
                //get the PID stuff too
                drive(0.6 * (position_tracker.getPoseEstimate().getY() + inches), Math.PI / 2.0, 0.4);
                position_tracker.update();
            }
        }
        position_tracker.setPoseEstimate(new Pose2d(current_pose.getX() + inches * Math.sin(angle), current_pose.getY() - inches * Math.cos(angle), angle));
    }

    public void moveForward(double inches) {
        position_tracker.update();
        Pose2d current_pose = position_tracker.getPoseEstimate();
        angle = current_pose.getHeading();
        position_tracker.setPoseEstimate(new Pose2d(0, 0, 0));

        current_error = 0;
        previous_error = 0;

        current_time = System.currentTimeMillis();
        previous_time = System.currentTimeMillis();

        if (inches > 0) {
            while (position_tracker.getPoseEstimate().getX() < inches - 0.1) {
                drive(0.6 * (inches - position_tracker.getPoseEstimate().getX()), 0.0, 0.4);
                position_tracker.update();
            }
        } else {
            while (position_tracker.getPoseEstimate().getX() > inches + 0.1) {
                drive(0.6 * (inches - position_tracker.getPoseEstimate().getX()), 0.0, 0.4);
                position_tracker.update();
            }
        }
        position_tracker.setPoseEstimate(new Pose2d(current_pose.getX() + inches * Math.cos(angle), current_pose.getY() + inches * Math.sin(angle), angle));
    }

    public void drive(double turning_factor, double speed) {
        for (int i = 0; i < 4; i++) {
            robot.wheel_list[i].setPower(speed * turning_factor * ((i > 1) ? -1 : 1));
        }
    }

    public void drive(double distance_factor, double offset, double speed) {
        double[] power = new double[4];
        for (int i = 0; i < 4; i++) {
            power[i] = getCorrection() * ((i > 1) ? -1 : 1) - distance_factor * (Math.cos(offset) + Math.sin(offset) * (i % 2 == 1 ? 1 : -1) * strafe);
        }
        double maximum = Math.max(1, Math.max(Math.max(Math.abs(power[0]), Math.abs(power[1])), Math.max(Math.abs(power[2]), Math.abs(power[3]))));
        for (int i = 0; i < 4; i++) {
            robot.wheel_list[i].setPower(power[i] / maximum * speed);
        }
    }

    public double getCorrection() {
        current_time = System.currentTimeMillis();
        current_error = modifiedAngle(angle - position_tracker.getPoseEstimate().getHeading());

        double p = current_error;
        double d = (current_error - previous_error) / (current_time - previous_time);

        previous_error = current_error;
        previous_time = current_time;

        return p_weight * p + d_weight * d;
    }

    public double modifiedAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < 0 - Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
    
}
