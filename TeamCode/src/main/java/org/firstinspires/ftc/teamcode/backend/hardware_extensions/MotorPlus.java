package org.firstinspires.ftc.teamcode.backend.hardware_extensions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.backend.control.low_level.PIDV2;

public class MotorPlus {
    private double kp_d, ki_d, kd_d;
    private double kp_a, ki_a, kd_a;

    private final static double wheel_radius = 0.0508, x02=.18, encoder_count_per_rev = 1120;

    private double distance_traversed;
    private int success_count = 0;
    private double last_encoder = 0.0, current_encoder;
    private double last_velocity = 0.0, current_vel=0.0, current_acc;
    private double current_power_cap;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor[] motors;

    private double pid_d_response, pid_acc_response, external_correction;

    private PIDV2 pid_d, pid_acc;


    public MotorPlus(DcMotor[] motors, double[] distance_values, double[] acceleration_values){
        // Use this when grouping motors logically (i.e, left drive motors might be controlled
        // using the same pid controll, so put them in the array).

        this.kp_d = distance_values[0];
        this.ki_d = distance_values[1];
        this.kd_d = distance_values[2];

        this.kp_a = acceleration_values[0];
        this.ki_a = acceleration_values[1];
        this.kd_a = acceleration_values[2];

        this.motors = motors;

        pid_acc = new PIDV2(this.kp_a, this.ki_a, this.kd_a) {
            @Override
            public void perform(double response) {
                // Yet to be decided how we will tackle the pid response here!
                double clipped_pid_response = Range.clip(response, -current_power_cap,
                        current_power_cap);

                // After processing, go ahead and power the robot
                setPid_acc_response(clipped_pid_response);
            }

            @Override
            public double getInputData() {
                return calculate_acceleration();
            }
        };

        pid_d = new PIDV2(this.kp_d, this.ki_d, this.kd_d) {
            @Override
            public void perform(double response) {
                // Yet to be decided how we will tackle the pid response here!
                double clipped_pid_response = Range.clip(response, -current_power_cap,
                        current_power_cap);

                // After processing, go ahead and power the robot
                setPid_d_response(clipped_pid_response);
            }

            @Override
            public double getInputData() {
                return current_encoder;
            }
        };

    }

    public double getMotorsArrayEncoderCount(){
        double encoder_sum = 0.0;
        for (DcMotor m:this.motors) encoder_sum += m.getCurrentPosition();
        double avg = encoder_sum / this.motors.length;
        return avg;
    }

    public void applyPower(double power){
        for(DcMotor m:motors){
            m.setPower(power);
        }
    }

    private double calculate_acceleration(){
        current_vel = (current_encoder - last_encoder)/runtime.milliseconds();
        current_acc = (current_vel-last_velocity)/runtime.milliseconds();
        last_velocity = current_vel;
        return current_acc;
    }

    private void setPid_d_response(double pid_d_response){
        this.pid_d_response = pid_d_response;
    }

    private void setPid_acc_response(double pid_acc_response){
        this.pid_acc_response = pid_acc_response;
    }

    public static boolean isWithin(double tolerance, double value, double target){
        return ((target-tolerance) < value) && (value < (target+tolerance));
    }

    public void drive_iter(double power_cap, double target_acc, double target_dist){
        // Update Encoder and other Values
        current_encoder = getMotorsArrayEncoderCount();
        current_power_cap = power_cap;
        runtime.reset();
        double target_encoder_value = distance_to_encoder(target_dist);

        // Execute PID
        pid_d.executePID(target_encoder_value);
        pid_acc.executePID(target_acc);

        // Check Success Condition
        if(isWithin(536, current_encoder, target_encoder_value)) {
            success_count += 1;
        }

        // Calculate the response
        double response;
        if(pid_d_response < 0){
            response = pid_d_response - Math.abs(pid_d_response * pid_acc_response)
                    - Math.abs(external_correction);
        } else {
            response = pid_d_response + Math.abs(pid_d_response * pid_acc_response)
                    + Math.abs(external_correction);
        }

        // Apply the power
        applyPower(response);

        // Update last_ values
        last_encoder = current_encoder;
    }

    public int getSuccessCount(){
        return success_count;
    }

    private double distance_to_encoder(double distance){
        return ((distance/(2*3.14*wheel_radius)) * encoder_count_per_rev);
    }

    public static double turn_distance(double angle_of_turn){
        return (0.0174533)*(x02 * angle_of_turn);
    }

    public void resetMotorInfo(){
        for (DcMotor m:motors){
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        current_encoder = 0.0;
        current_power_cap = 0.0;
        runtime.reset();
        pid_d.restartPID();
        pid_acc.restartPID();
        success_count = 0;
        last_encoder = 0;
        pid_acc_response = 0;
        pid_d_response = 0;
        current_vel = 0.0;
        current_acc = 0.0;
    }

    private double getLast_velocity() {
        return last_velocity;
    }

    public double getCurrent_acc() {
        return current_acc;
    }

    public void changePIDParam(double[] distance_values, double[] acceleration_values){
        this.kp_d = distance_values[0];
        this.ki_d = distance_values[1];
        this.kd_d = distance_values[2];

        this.kp_a = acceleration_values[0];
        this.ki_a = acceleration_values[1];
        this.kd_a = acceleration_values[2];
    }

    private double getCurrent_vel() {
        return current_vel;
    }

    public void add_correction(double external_correction){
        this.external_correction = external_correction;
    }
}
