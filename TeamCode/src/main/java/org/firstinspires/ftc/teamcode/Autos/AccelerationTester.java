package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.FSM.State;
import org.firstinspires.ftc.teamcode.HardwareMap.HMap;
import org.firstinspires.ftc.teamcode.HardwareMap.IMUPlus;
import org.firstinspires.ftc.teamcode.HardwareMap.MotorPlus;

@Autonomous(name = "AccelerationTester")
public class AccelerationTester extends LinearOpMode {
    HMap robot = new HMap();

    double[] distance_pid_values = new double[]{2.832556, 0.00007985467, 0.00008};
//    double[] acc_pid_values      = new double[]{0.000012974, 0.000000038723, 0.00000000084065};
double[] acc_pid_values      = new double[]{0, 0, 0};

    double[] imu_pid_values      = new double[]{0.00000208, 0.0000000000000532873,
            0.00000000000000000093073}; //ki_old = 0.000000000000792873, kd_old = 0.000000000000480373
    
    int success_cap = 50;
    MotorPlus TL, TR, BL, BR;
    IMUPlus imu;

    private State BULK_READ = new State() {
        @Override
        public void state_action() {
            TL.getMotorsArrayEncoderCount();
            TR.getMotorsArrayEncoderCount();
            BL.getMotorsArrayEncoderCount();
            BR.getMotorsArrayEncoderCount();
            imu.getAngle();
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        TL = new MotorPlus(new DcMotor[]{robot.TL}, distance_pid_values, acc_pid_values);
        TR = new MotorPlus(new DcMotor[]{robot.TR}, distance_pid_values, acc_pid_values);
        BL = new MotorPlus(new DcMotor[]{robot.BL}, distance_pid_values, acc_pid_values);
        BR = new MotorPlus(new DcMotor[]{robot.BR}, distance_pid_values, acc_pid_values);
        imu = new IMUPlus(robot.imu, imu_pid_values);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Done...");
        telemetry.update();

        waitForStart();
        imu.resetAngle();
        drive(0.6, 0.090024, 0.8);
        sleep(500);
        drive(0.6, 0.090024, -0.8);
        sleep(500);
        turnLeft(0.6, 0.090024, 90.5);
        sleep(500);
        turnRight(0.6, 0.090024, 90.5);
    }

    public void drive(double power_cap, double acceleration, double distance){
        imu.resetAngle();
        TL.resetMotorInfo();
        TR.resetMotorInfo();
        BL.resetMotorInfo();
        BR.resetMotorInfo();
        while((opModeIsActive() && !isStopRequested())
                && ((TL.getSuccessCount() < success_cap)) || (TR.getSuccessCount() < success_cap)){
            FiniteStateMachine.change_state(BULK_READ);

            // IMU PID STUFF
            imu.imu_iter(0.0001);
            double imu_correction = imu.getCorrection();
            telemetry.addData("IMU Correction -> ", imu_correction);
            telemetry.addData("Global Angle   ->", imu.getGlobalAngle());
            telemetry.update();
            if (!MotorPlus.isWithin(0.03, imu_correction, 0.0)){
                // If the correction is negative, we past the angle
                TL.add_correction(imu_correction);
                TR.add_correction(-imu_correction);
                BL.add_correction(imu_correction);
                BR.add_correction(-imu_correction);
            } else {
                TL.add_correction(0.0);
                TR.add_correction(0.0);
                BL.add_correction(0.0);
                BR.add_correction(0.0);
            }

            // Calculating accelerations
            TL.drive_iter(power_cap, acceleration, distance);
            TR.drive_iter(power_cap, acceleration, distance);
            BL.drive_iter(power_cap, acceleration, distance);
            BR.drive_iter(power_cap, acceleration, distance);
        }
    }

    public void turnLeft(double power_cap, double acceleration, double angle_of_turn){
        imu.resetAngle();
        TL.resetMotorInfo();
        TR.resetMotorInfo();
        BL.resetMotorInfo();
        BR.resetMotorInfo();
        double turn_dist = MotorPlus.turn_distance(angle_of_turn);
        while((opModeIsActive() && !isStopRequested())
                && ((TL.getSuccessCount() < success_cap)) || (TR.getSuccessCount() < success_cap)){
            FiniteStateMachine.change_state(BULK_READ);

            // IMU PID STUFF
            imu.imu_iter(0.0001);
            double imu_correction = imu.getCorrection();
            telemetry.addData("IMU Correction -> ", imu_correction);
            telemetry.addData("Global Angle   ->", imu.getGlobalAngle());
            telemetry.update();
            if (!MotorPlus.isWithin(0.03, imu_correction, 0.0)){
                // If the correction is negative, we past the angle
                TL.add_correction(imu_correction);
                TR.add_correction(-imu_correction);
                BL.add_correction(imu_correction);
                BR.add_correction(-imu_correction);
            } else {
                TL.add_correction(0.0);
                TR.add_correction(0.0);
                BL.add_correction(0.0);
                BR.add_correction(0.0);
            }

            // Calculating accelerations
            TL.drive_iter(power_cap, acceleration, turn_dist);
            TR.drive_iter(power_cap, acceleration, -turn_dist);
            BL.drive_iter(power_cap, acceleration, turn_dist);
            BR.drive_iter(power_cap, acceleration, -turn_dist);
        }
    }

    public void turnRight(double power_cap, double acceleration, double angle_of_turn){
        imu.resetAngle();
        TL.resetMotorInfo();
        TR.resetMotorInfo();
        BL.resetMotorInfo();
        BR.resetMotorInfo();
        double turn_dist = MotorPlus.turn_distance(angle_of_turn);
        while((opModeIsActive() && !isStopRequested())
                && ((TL.getSuccessCount() < success_cap)) || (TR.getSuccessCount() < success_cap)){
            FiniteStateMachine.change_state(BULK_READ);

            // IMU PID STUFF
            imu.imu_iter(0.0001);
            double imu_correction = imu.getCorrection();
            telemetry.addData("IMU Correction -> ", imu_correction);
            telemetry.addData("Global Angle   ->", imu.getGlobalAngle());
            telemetry.update();
            if (!MotorPlus.isWithin(0.03, imu_correction, 0.0)){
                // If the correction is negative, we past the angle
                TL.add_correction(imu_correction);
                TR.add_correction(-imu_correction);
                BL.add_correction(imu_correction);
                BR.add_correction(-imu_correction);
            } else {
                TL.add_correction(0.0);
                TR.add_correction(0.0);
                BL.add_correction(0.0);
                BR.add_correction(0.0);
            }

            // Calculating accelerations
//            TL.drive_iter(power_cap, acceleration, -turn_dist);
//            TR.drive_iter(power_cap, acceleration, turn_dist);
//            BL.drive_iter(power_cap, acceleration, -turn_dist);
//            BR.drive_iter(power_cap, acceleration, turn_dist);
        }
    }
}
