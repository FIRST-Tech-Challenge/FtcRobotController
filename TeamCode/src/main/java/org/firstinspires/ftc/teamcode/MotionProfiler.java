package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDFController;

public class MotionProfiler {

    double max_v, max_a, new_max_v, start_pos, end_pos, distance, total_dt, accel_dt, accel_dist, cruise_dist, cruise_dt,
            decel_time;
    double cruiseCurrentDt;
    public boolean isDone=false;
    public MotionProfiler(double max_v, double max_a, double start_pos, double end_pos) {
        this.max_v = max_v;
        this.max_a = max_a;
        this.start_pos = start_pos;
        this.end_pos=end_pos;
        distance =Math.abs(end_pos-start_pos);

        if(end_pos-start_pos<0) {
            this.max_v= -this.max_v;
            this.max_a = -this.max_a;
        }
    }

    public void init() {
        /*acceleration starts at the beginning of the trajectory.
        cruising starts at accel_dt seconds, ends at cruise_dt seconds
        decel starts at decel_time seconds.
        */

        distance = end_pos-start_pos;

        // calculate the time it takes to accelerate to max velocity
        accel_dt = max_v / max_a;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        accel_dist = 0.5 * max_a * Math.pow(accel_dt,2);

        if (accel_dist > halfway_distance)
            accel_dt = Math.sqrt(halfway_distance / (0.5 * max_a));

        accel_dist = 0.5 * max_a * Math.pow(accel_dt,2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        new_max_v = max_a * accel_dt;

        //calculate the distance and time that we're cruising at max velocity
        cruise_dist = distance - 2 * accel_dist;
        cruise_dt = cruise_dist / new_max_v;
        decel_time = accel_dt + cruise_dt;
        //we should be at decel_time seconds when deceleration starts.

        total_dt = 2*accel_dt + cruise_dt;
    }

    public double profile_pos(double current_dt) {
        //Returns where the mechanism should be based on the profile

        if(current_dt>total_dt) {
            isDone = true;
            return end_pos;
        }

        //if we're accelerating:
        if(current_dt < accel_dt) return start_pos + 0.5 *max_a * Math.pow(current_dt,2);

        //cruising:
        if(current_dt < decel_time) {
            double cruising_dt = current_dt - accel_dt;
            return start_pos + accel_dist + new_max_v * cruising_dt;
        }

        //decelerating:
        return start_pos + accel_dist + cruise_dist + new_max_v * decel_time - 0.5 * max_a * Math.pow(decel_time,2);

    }

}