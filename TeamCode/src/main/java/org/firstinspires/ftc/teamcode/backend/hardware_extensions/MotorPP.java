package org.firstinspires.ftc.teamcode.backend.hardware_extensions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.backend.control.low_level.PIDV2;

public class MotorPP {

    // PID PARAMETERS
    private double dkp, dki, dkd; // Distance PID Param
    private double akp, aki, akd; // Acceleration PID Param
    private double vkp, vki, vkd; // Velocity PID Param

    private final static double wheel_radius = 0.0508, x02=.18, encoder_count_per_rev = 1120;

    private double distance_traversed; // Distance Traversed
    private double current_velocity, current_encoder, current_acc; // Motor's current state
    private double last_velocity, last_encoder, last_acc;
    private DcMotor[] motors; // Logical Motor Group

    private int success_tolerance = 636;
    private int success_count = 0;

    private double dResponse, aResponse, vResponse, external_response;

    private PIDV2 dPid, aPid, vPid;

    private ElapsedTime runtime = new ElapsedTime();

    public MotorPP(DcMotor[] motors, double[] distance_pid, double[] acceleration_pid,
                   double[] velocity_pid){
        // This constructor assumes control of the motors with built in pid performance
        // In other words, this is plug and play
        runtime.reset();
        this.motors = motors;

        this.dkp = distance_pid[0];
        this.dki = distance_pid[1];
        this.dkd = distance_pid[2];

        this.akp = acceleration_pid[0];
        this.aki = acceleration_pid[1];
        this.akd = acceleration_pid[2];

        this.vkp = velocity_pid[0];
        this.vki = velocity_pid[1];
        this.vkd = velocity_pid[2];

        dPid = new PIDV2(dkp, dki, dkd) {
            @Override
            public void perform(double response) {
                setdResponse(response);
            }

            @Override
            public double getInputData() {
                return getCurrentPosition();
            }
        };

        aPid = new PIDV2(akp, aki, akd) {
            @Override
            public void perform(double response) {
                setaResponse(response);
            }

            @Override
            public double getInputData() {
                return calc_acc();
            }
        };

        vPid = new PIDV2(vkp, vki, vkd) {
            @Override
            public void perform(double response) {
                setvResponse(response);
            }

            @Override
            public double getInputData() {
                return calc_vel();
            }
        };
    }

    public MotorPP(DcMotor[] motors, double[] distance_pid, double[] acceleration_pid,
                   double[] velocity_pid, PIDV2 dpid, PIDV2 aPid, PIDV2 vPid){
        // this one lets you define how you want to create your pid controllers
        runtime.reset();
        this.motors = motors;

        this.dkp = distance_pid[0];
        this.dki = distance_pid[1];
        this.dkd = distance_pid[2];

        this.akp = acceleration_pid[0];
        this.aki = acceleration_pid[1];
        this.akd = acceleration_pid[2];

        this.vkp = velocity_pid[0];
        this.vki = velocity_pid[1];
        this.vkd = velocity_pid[2];

        this.dPid = dpid;
        this.aPid = aPid;
        this.vPid = vPid;
    }

    public void setPower(double power){
        for (DcMotor m:motors){
            m.setPower(power);
        }
    }

    private double getCurrentPosition(){
        double encoder_sum = 0;
        for (DcMotor m : motors) {
            encoder_sum += m.getCurrentPosition();
        }
        double avg = encoder_sum / motors.length;
        current_encoder = avg;
        return avg;
    }

    private double calc_vel(){
        current_velocity = (current_encoder - last_encoder)/runtime.milliseconds();
        return current_velocity;
    }

    private double calc_acc(){
        current_acc = (current_velocity - last_velocity)/runtime.milliseconds();
        return current_acc;
    }

    private void setdResponse(double dResponse) {
        this.dResponse = dResponse;
    }

    private void setaResponse(double aResponse) {
        this.aResponse = aResponse;
    }

    private void setvResponse(double vResponse) {
        this.vResponse = vResponse;
    }

    public void actuate(double power_cap, double target_distance, double target_acc,
                        double target_vel){
        // This is the code that iterates through each pid iteration
        runtime.reset();

        // Variables are updated in their iteration of pids (i.e, current_encoder is updated in the
        // loop so its not repeated).
        dPid.executePID(calculate_distance(target_distance));
        vPid.executePID(target_vel);
        aPid.executePID(target_acc);

        double raw_response = dResponse + vResponse + aResponse + external_response;
        double clipped_response = Range.clip(raw_response, -power_cap, power_cap);

        // Check Success Condition
        if(isWithin(success_tolerance, current_encoder, target_distance)) {
            success_count += 1;
        }

        setPower(clipped_response);

        // Resets the variables
        last_encoder = current_encoder;
        last_velocity = current_velocity;
        last_acc = current_acc;
    }

    public static boolean isWithin(double tolerance, double value, double target){
        return ((target-tolerance) < value) && (value < (target+tolerance));
    }

    // Getters and Setters

    public double getCurrent_encoder() {
        return current_encoder;
    }

    public double getCurrent_acc() {
        return current_acc;
    }

    public double getCurrent_velocity() {
        return current_velocity;
    }

    public int getSuccess_count() {
        return success_count;
    }

    public void addExternalResponse(double response){
        this.external_response = response;
    }

    private double calculate_distance(double target_distance){
        return ((target_distance/(2*3.14*wheel_radius)) * encoder_count_per_rev);
    }

    public void resetMotor(){
        current_acc = 0.0;
        current_velocity = 0.0;
        current_encoder = 0.0;
        last_acc = 0.0;
        last_velocity = 0.0;
        last_encoder = 0.0;
        runtime.reset();
        aPid.restartPID();
        dPid.restartPID();
        vPid.restartPID();
        success_count = 0;
    }
}
