package org.firstinspires.ftc.teamcode.classes;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class MotionProfile {

    double acceleration = 0;
    double halfDistance = 0;
    double accelerationDistance = 0;
    double deacceleration = 0;
    double deaccelerationTime = 0;
    double cruiseDistance = 0;
    double cruise_dt = 0;
    double entire_dt = 0;
    double cruise_current_dt = 0;

    Motor motor = null;

    double max_acceleration = 0;
    double max_velocity = 0;
    double distance = 0;
    double current_dt = 0;

    public MotionProfile(Motor motor) {
        this.motor = motor;
    }

    public double calculate(double max_acceleration, double max_velocity, double distance, double current_dt) {
        // calculate the time it takes to accelerate to max velocity
        acceleration = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        halfDistance = distance / 2;
        accelerationDistance = 0.5 * max_acceleration * (acceleration * acceleration);

        if (accelerationDistance > halfDistance) {
            acceleration = Math.sqrt(halfDistance / (0.5 * max_acceleration));
        }

        accelerationDistance = 0.5 * max_acceleration * acceleration *  acceleration;

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration;

        // we decelerate at the same rate as we accelerate
        deacceleration = acceleration;

        // calculate the time that we're at max velocity
        cruiseDistance = distance - 2 * accelerationDistance;
        cruise_dt = cruiseDistance / max_velocity;
        deaccelerationTime = acceleration + cruise_dt;

        // check if we're still in the motion profile
        entire_dt = acceleration + cruise_dt + deacceleration;
        if (current_dt > entire_dt){
            return distance;
        }

        // if we're accelerating
        if (current_dt < acceleration) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * current_dt * current_dt;
        }

        // if we're cruising
        else if (current_dt < deaccelerationTime) {
            accelerationDistance = 0.5 * max_acceleration * acceleration * acceleration;
            cruise_current_dt = current_dt - acceleration;

            // use the kinematic equation for constant velocity
            return accelerationDistance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            accelerationDistance = 0.5 * max_acceleration * acceleration * acceleration;
            cruiseDistance = max_velocity * cruise_dt;
            deaccelerationTime = current_dt - deaccelerationTime;

            // use the kinematic equations to calculate the instantaneous desired position
            return accelerationDistance + cruiseDistance + max_velocity * deaccelerationTime - 0.5 * max_acceleration * deaccelerationTime * deaccelerationTime;
        }
    }

}
