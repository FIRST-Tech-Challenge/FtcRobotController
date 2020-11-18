package org.firstinspires.ftc.teamcode.MISC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MISC.CONSTANTS;
import org.firstinspires.ftc.teamcode.PID.PIDV2;

public class DeadReckoning {
    // Positional Information
    double pX=0, pY=0;

    // Telemetry
    public Telemetry telem;

    // Misc. Vars
    double power_cap = 0.6;

    // IMU Variables
    BNO055IMU[] imu_list;
    double imu_pos,      imu_vel,      imu_acc,      imu_global_angle;
    double last_imu_pos, last_imu_vel, last_imu_acc;
    Orientation lastAngles = new Orientation();

    // Encoder Variables
    DcMotor[] L, R;
    double left_enc,      right_enc;
    double last_left_enc, last_right_enc;

    // PID Accessible Targets
    double target_angle = 0, target_position=0;
    double target_x = 0, target_y = 0;

    // PID Responses
    double imu_response      = 0;
    double distance_response = 0;

    // PID Objects
    PIDV2 imu_pid = new PIDV2(CONSTANTS.DeadRImuPid_) {
        @Override
        public void perform(double response) {
            imu_response = response;
        }

        @Override
        public double getInputData() {
            return imu_global_angle;
        }
    };

    PIDV2 distance_pid = new PIDV2(CONSTANTS.DeadRDPid_) {
        @Override
        public void perform(double response) {
            distance_response = response;
        }

        @Override
        public double getInputData() {
            // returns the average encoder count
            double avg_enc = (left_enc + right_enc) / 2.0;
            return encoder_to_distance(avg_enc); // input to dpid is distance in meters
        }
    };

    // Timekeeping
    ElapsedTime imu_timing = new ElapsedTime();

    public DeadReckoning(DcMotor[] leftDriveMotors, DcMotor[] rightDriveMotors,
                         BNO055IMU[] imuList, Telemetry telemetry){
        // Constructor for our deadreckoning library
        this.L = leftDriveMotors;
        this.R = rightDriveMotors;
        this.imu_list = imuList;
        this.telem = telemetry;
    }

    public void move(double x, double y){
        target_x = x;
        target_y = y;

        calculate_target_angle(target_x, target_y);
        calculate_target_pos(target_x, target_y);

        while (!stopCondition()){
            // Update Sensor Information
            updateIMU();
            updateAllMotors();

            // Calculate our current position
            process_current_position_data();

            // Calculate the target (and update them)


            // execute pid's with their respective targets
            imu_pid.executePID(target_angle);
            distance_pid.executePID(target_position);

            telem.addData("Target Angle", target_angle);
            telem.addData("Target Distance, ", target_position);
            telem.addData("IMU Response: ", imu_response);
            telem.addData("Distance Response: ", distance_response);
            telem.addData("Target X: ", target_x);
            telem.addData("Target Y: ", target_y);
            telem.addData("Global: ", imu_global_angle);
            telem.update();

            // send response to motors
            for (DcMotor m : L){
//                m.setPower(1);
                m.setPower(distance_response - imu_response);
            }

            for (DcMotor m : R){
//                m.setPower(1);
                m.setPower(distance_response + imu_response);
            }
        }
    }

    private boolean stopCondition(){
        // TODO: Find out a good stop condition!
        return false;
    }

    private void calculate_target_angle(double x, double y){
        // Calculate target orientation
        double dx, dy;
        dx = x - pX;
        dy = y - pY;

        target_angle = imu_global_angle + Math.toDegrees(Math.atan((dy/dx))); // updates the target angle
    }

    private void calculate_target_pos(double x, double y){
        // Calculate target displacement
        double dx, dy;
        dx = x - pX;
        dy = y - pY;

        target_position = calcHypotenuse(dy, dx);
    }

    // Process Position Data
    private void process_current_position_data(){
        // Gather information about position and update the position value of robot
        double imuPx, imuPy; // Should be in meters
        double angle_in_radians = convert_to_radians(imu_global_angle);

        double encPx, encPy;

        // IMU Position Estimate
        imuPy = imu_pos * Math.sin(angle_in_radians);
        imuPx = imu_pos * Math.cos(angle_in_radians);

        // Encoder Position Estimates
        double avg_enc = (left_enc + right_enc)/(double)2.0;

        // Calculate Encoder Displacement using Angle
        encPy = avg_enc * Math.sin(angle_in_radians);
        encPx = avg_enc * Math.cos(angle_in_radians);

        // Do final conversions (to meters)
        encPy = encoder_to_distance(encPy);
        encPx = encoder_to_distance(encPx);

        // Avg both position inputs
        pX += (imuPx + encPx)/(double)2.0;
        pY += (imuPy + encPy)/(double)2.0;
    }

    // IMU Updates
    public void updateIMU(){
        double imu_acc_t = 0, imu_vel_t = 0, imu_pos_t = 0;
        double time_ = imu_timing.milliseconds();

        for (BNO055IMU imu : imu_list){
            imu_acc_t += updateImuAcc(imu);
            imu_vel_t += updateImuVel(time_);
            imu_pos_t += updateImuPos(time_);
            imu_global_angle = updateImuGlobalAngle(imu);
        }

        imu_acc_t /= (double)imu_list.length;
        imu_vel_t /= (double)imu_list.length;
        imu_pos_t /= (double)imu_list.length;

        imu_acc = imu_acc_t;
        imu_vel = imu_vel_t;
        imu_pos = imu_pos_t;

        imu_timing.reset();
    }

    private double updateImuAcc(BNO055IMU imu){
        Acceleration acc = imu.getAcceleration();
        acc.toUnit(DistanceUnit.METER);
        return calcHypotenuse(acc.yAccel, acc.xAccel);
    }

    private double updateImuVel(double time_){
        return imu_acc*time_;
    }

    private double updateImuPos(double time_){
        return imu_vel*time_;
    }

    private double updateImuGlobalAngle(BNO055IMU imu){
        return getAngle(imu);
    }

    private double getAngle(BNO055IMU imu) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        imu_global_angle += deltaAngle;
        lastAngles = angles;
        return imu_global_angle;
    }

    public void resetImu(){
        resetImuAcc();
        resetImuVel();
        resetImuPos();
        resetImuAngle(imu_list[0]);
    }

    private void resetImuAcc(){
        last_imu_acc = imu_acc;
    }

    private void resetImuVel(){
        last_imu_vel = imu_vel;

    }

    private void resetImuPos(){
        last_imu_pos = imu_pos;
    }

    private void resetImuAngle(BNO055IMU imu){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        imu_global_angle = 0;
    }

    // Encoder Updates
    public void updateAllMotors(){
        updateLeftDrive();
        updateRightDrive();
    }

    public void updateLeftDrive(){
        int enc_ = 0;
        for (DcMotor m: L){
            enc_ += getMotorEnc(m);
        }
        left_enc = (enc_/(double)L.length);
    }

    public void updateRightDrive(){
        int enc_ = 0;
        for (DcMotor m: R){
            enc_ += getMotorEnc(m);
        }
        right_enc = (enc_/(double)R.length);
    }

    private int getMotorEnc(DcMotor m){
        return m.getCurrentPosition();
    }

    public void resetLeftDriveEnc(){
        last_left_enc = left_enc;
    }

    public void resetRightDriveEnc(){
        last_right_enc = right_enc;
    }

    // MISC FUNCTIONS
    private double calcHypotenuse(double a, double b){
        return Math.sqrt(((a*a) + (b*b)));
    }

    private double encoder_to_distance(double encoder_count){
        // TODO: Create Formula for the encoder to distance conversion
        return 1.2;
    }

    private double convert_to_radians(double degree){
        return degree*0.0174533;
    }
}
