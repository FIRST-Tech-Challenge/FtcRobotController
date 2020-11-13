package org.firstinspires.ftc.teamcode.Autos.BASE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMap.HMap;
import org.firstinspires.ftc.teamcode.HardwareMap.MotorPlus;
import org.firstinspires.ftc.teamcode.MISC.CONSTANTS;

public abstract class AutoBase extends LinearOpMode {

    HMap robot = new HMap();

    public abstract void autoCode();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initializing the hardware map
        robot.init(hardwareMap);

        // Calibrating Sensors
        addTelemetryData("IMU Calibration: ",
                Boolean.toString(robot.imu.isGyroCalibrated()));
        calibrateIMU();
        addTelemetryData("IMU Calibration: ",
                Boolean.toString(robot.imu.isGyroCalibrated()));

        // Pre-Auto Calibration
        robot.imu.resetAngle();

        // End of Initialization
        waitForStart();

        autoCode();
    }

    private void addTelemetryData(String caption, String msg){
        telemetry.addData(caption, msg);
        telemetry.update();
    }

    private void calibrateIMU(){
        // Code for calibrating the imu
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
    }

    public void drive(double power_cap, double target_distance){
        // General Drive Method using recommended acceleration and velocity (this should make your life easier)
        drive(power_cap, target_distance, CONSTANTS.target_acceleration, CONSTANTS.target_velocity);
    }

    public void turn(double power_cap, double target_angle){
        // General Turn Method using Recommended Values
        turn(power_cap, target_angle, CONSTANTS.target_acceleration, CONSTANTS.target_velocity);
    }

    public void turn(double power_cap, double target_angle, double target_acceleration,
                     double target_velocity) {
        // Resetting the parameters for our controller
        int target_distance = (int)((28/9)*target_angle); 
        robot.imu.resetAngle();
        robot.TL.resetMotor();
        robot.TR.resetMotor();
        robot.BL.resetMotor();
        robot.BR.resetMotor();

        // Required Stop Condition
        int success_count = robot.TL.getSuccess_count();

        // "PID" Loop
        while ((opModeIsActive() && !isStopRequested()) && (success_count < 90)) {
            success_count = robot.TL.getSuccess_count();
            // IMU PID STUFF
            robot.imu.imu_iter(0.00000001); // target angle can't be zero!
            double imu_correction = robot.imu.getCorrection();

            if (!MotorPlus.isWithin(0.0195, imu_correction, 0.0)) {
                // If correction is needed, we correct the angle
                robot.TL.addExternalResponse(-imu_correction);
                robot.TR.addExternalResponse(imu_correction);
                robot.BL.addExternalResponse(-imu_correction);
                robot.BR.addExternalResponse(imu_correction);
            } else {
                // If withing tolerance, set motor power to zero
                robot.TL.addExternalResponse(0.0);
                robot.TR.addExternalResponse(0.0);
                robot.BL.addExternalResponse(0.0);
                robot.BR.addExternalResponse(0.0);
            }

            // Supplying the "targets" to the Motors
            robot.TL.actuate(power_cap, -target_distance, target_acceleration, target_velocity);
            robot.TR.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            robot.BL.actuate(power_cap, -target_distance, target_acceleration, target_velocity);
            robot.BR.actuate(power_cap, target_distance, target_acceleration, target_velocity);
        }
    }

    public void drive(double power_cap, double target_distance, double target_acceleration,
                      double target_velocity) {
        // Code for making the robot drive
        /*
        - Currently supports forward and backward movement
         */
        // Resetting the parameters for our controller
        robot.TL.resetMotor();
        robot.TR.resetMotor();
        robot.BL.resetMotor();
        robot.BR.resetMotor();
        robot.imu.resetAngle();

        // Required Stop Condition
        int success_count = robot.TL.getSuccess_count();

        // "PID" Loop
        while ((opModeIsActive() && !isStopRequested()) && (success_count < 90)) {
            success_count = robot.TL.getSuccess_count();
            // IMU PID STUFF
            robot.imu.imu_iter(0.00000001); // target angle can't be zero!
            double imu_correction = robot.imu.getCorrection();

            if (!MotorPlus.isWithin(0.0195, imu_correction, 0.0)) {
                // If correction is needed, we correct the angle
                robot.TL.addExternalResponse(-imu_correction);
                robot.TR.addExternalResponse(imu_correction);
                robot.BL.addExternalResponse(-imu_correction);
                robot.BR.addExternalResponse(imu_correction);
            } else {
                // If withing tolerance, set motor power to zero
                robot.TL.addExternalResponse(0.0);
                robot.TR.addExternalResponse(0.0);
                robot.BL.addExternalResponse(0.0);
                robot.BR.addExternalResponse(0.0);
            }

            // Supplying the "targets" to the Motors
            robot.TL.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            robot.TR.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            robot.BL.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            robot.BR.actuate(power_cap, target_distance, target_acceleration, target_velocity);
        }
    }
}
