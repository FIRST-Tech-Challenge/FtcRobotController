package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.MISC.CONSTANTS;
import org.firstinspires.ftc.teamcode.PID.PIDV2;

import java.util.List;

public class DriveTrain {

    private DcMotor[] driveMotors; // 1D array with TL, TR, BL, BR arrangement (position matters!)
    private DcMotor[] leftDriveMotors, rightDriveMotors; // specific side of motors
    private List<BNO055IMU> imuList;
    private List<DcMotor> externalEncodersList;

    private ElapsedTime imu_pos_timer = new ElapsedTime();

    // Casts as MotorPP
    public MotorPP L, R;

    // IMU Related Functions
    private double p_imu = 0, p_imu_last = 0;
    private double power_cap = 1.0;

    // Encoder Related Functions
    private double enc_left = 0, enc_left_last = 0, enc_right = 0, enc_right_last = 0;

    // Creates PIDV2 objects
    private PIDV2 dPid = new PIDV2(CONSTANTS.fusedDPid_) {
        @Override
        public void perform(double response) {
            double clipped_response = Range.clip(response, -power_cap, power_cap);
            L.setPower(clipped_response);
            R.setPower(clipped_response);
        }

        @Override
        public double getInputData() {
            // made relative to last call (i.e, encoders are pseudo-reset to zero)
            return getRelativePosition();
        }
    };

    private PIDV2 aPid = new PIDV2(0, 0 ,0) {
        @Override
        public void perform(double response) {

        }

        @Override
        public double getInputData() {
            return 0;
        }
    };

    private PIDV2 vPid = new PIDV2(0, 0, 0) {
        @Override
        public void perform(double response) {

        }

        @Override
        public double getInputData() {
            return 0;
        }
    };

    public DriveTrain(DcMotor[] driveMotors){
        this.driveMotors = driveMotors;
        assert this.driveMotors != null; // Makes sure that array is not empty
        this.leftDriveMotors = new DcMotor[]{driveMotors[0], driveMotors[2]}; // TL, BL
        this.rightDriveMotors = new DcMotor[]{driveMotors[1], driveMotors[3]}; // TR, BR

        // Creates the new MotorPP versions
        L = new MotorPP(this.leftDriveMotors, CONSTANTS.dPid_, CONSTANTS.aPid_, CONSTANTS.vPid_,
                dPid, aPid, vPid);
        R = new MotorPP(this.rightDriveMotors, CONSTANTS.dPid_, CONSTANTS.aPid_, CONSTANTS.vPid_,
                dPid, aPid, vPid);
    }

    public void addDistanceInput(BNO055IMU imu){
        // Add the imu to our imuList where it can be accessed by distance approximators
        imuList.add(imu);
    }
    public void addDistanceInput(DcMotor motor){
        // Add the distance to our 'distance' arrays (by default drivetrain uses drive motors for
        // encoder based localization so you shouldn't add them here)

        externalEncodersList.add(motor);
    }

    public void calculatePositionFromImu(){
        //Misc Variables
        double velocity_in_x = 0, velocity_in_y = 0;
        double change_position_x, change_position_y;
        double time_ = imu_pos_timer.milliseconds(); // represents time since method last called

        for (BNO055IMU imu: imuList){
            // get's the instantaneous acceleration
            Acceleration acc = imu.getAcceleration();

            // Casting to proper units (can be changed)
            acc.toUnit(DistanceUnit.METER);

            //Gets the proper components of the acceleration
            double xAccel = acc.xAccel;
            double yAccel = acc.yAccel;

            // Integrates the acceleration for velocity
            double xVel = xAccel*time_;
            double yVel = yAccel*time_;

            // Adds the velocity to be averaged in global scope
            velocity_in_x += xVel;
            velocity_in_y += yVel;
        }
        // Average all values (typecast to double done to avoid integer division)
        velocity_in_x /= (double)imuList.size();
        velocity_in_y /= (double)imuList.size();

        // Integrate Velocities to get change in position
        change_position_x = velocity_in_x*time_;
        change_position_y = velocity_in_y*time_;

        // Add to new position values (from imu) using pythagorean theorem
        p_imu += Math.sqrt( ((change_position_y*change_position_y) + (change_position_x*change_position_x)) );


        // Reset time for next method call
        imu_pos_timer.reset();
    }

    private void calculatePosFromEncoders(){
        int temp_enc_left = 0, temp_enc_right = 0;

        for (DcMotor m: leftDriveMotors){
            // Adds the values to the left encoders global position
            temp_enc_left += m.getCurrentPosition();
        }

        for (DcMotor m: rightDriveMotors){
            // Adds the values to the right encoders global position
            temp_enc_right += m.getCurrentPosition();
        }

        // Averages both of them
        temp_enc_left *= 0.5;
        temp_enc_right *= 0.5;

        enc_left += temp_enc_left;
        enc_right += temp_enc_right;
    }

    public double getRelativePosition(){
        // Returns position from last pseudo-reset (assumes straight line)
        // Get imu_position
        double position_imu = p_imu - p_imu_last;

        // Convert position to encoders
        double pos_in_encoders = ( (position_imu * CONSTANTS.encoder_count_per_rev_REV)
                /(2*3.14*CONSTANTS.wheel_radius_meters_MECANUM) );

        // Get Relative Position from wheels
        double enc_left_new = enc_left - enc_left_last;
        double enc_right_new = enc_right - enc_right_last;

        // Average them all
        double fused_pos = (pos_in_encoders + enc_left_new + enc_right_new)/3.0;
        return fused_pos;
    }

    public void pseudo_reset(){
        pseudo_reset_encoders();
        pseudo_reset_imu();
    }

    private void pseudo_reset_encoders(){
        // This is like a reset but only on the software level
        enc_right_last = enc_right;
        enc_left_last = enc_left;
    }

    private void pseudo_reset_imu(){
        p_imu_last = p_imu;
    }

    public void drive(double power, double target_distance){
        // TODO: Add Stop Condition (either isActive() or !isStopRequested())
        // set the power cap
        power_cap = power;

        // Reset our variables
        pseudo_reset();

        // Calculate target Encoders
        double target_encoders = ( (target_distance * CONSTANTS.encoder_count_per_rev_REV)
                /(2*3.14*CONSTANTS.wheel_radius_meters_MECANUM) );

        // Enter dPid Loop
        while(true){
            dPid.executePID(target_encoders);
        }

    }
}
