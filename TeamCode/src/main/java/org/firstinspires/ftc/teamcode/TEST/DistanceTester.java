package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.middleend.HardwareMappings.HMap;
import org.firstinspires.ftc.teamcode.backend.hardware_extensions.IMUPlus;
import org.firstinspires.ftc.teamcode.backend.hardware_extensions.MotorPP;
import org.firstinspires.ftc.teamcode.backend.hardware_extensions.MotorPlus;

@Autonomous(name="MotorPP Distance(0.40)")
@Disabled
@Deprecated
public class DistanceTester extends LinearOpMode {
    HMap robot = new HMap();
    IMUPlus imu;
    MotorPP TL, TR, BL, BR;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

//        imu = new IMUPlus(robot.imu, CONSTANTS.imuPid_);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Done...");
        telemetry.update();

//        TL = new MotorPP(new DcMotor[]{robot.TL}, dPid_, aPid_, vPid_);
//        TR = new MotorPP(new DcMotor[]{robot.TR}, dPid_, aPid_, vPid_);
//        BL = new MotorPP(new DcMotor[]{robot.BL}, dPid_, aPid_, vPid_);
//        BR = new MotorPP(new DcMotor[]{robot.BR}, dPid_, aPid_, vPid_);

        waitForStart();
        imu.resetAngle();

        // Actual Driving + Moving Code Goes here!
        drive(1.0, 0.40, 0.0000124, .114);
        turn(1.0, 90, 0.0000124, .114);
    }

    public void drive(double power_cap, double target_distance, double target_acceleration,
                double target_velocity) {
        // Resetting the parameters for our controller
        TL.resetMotor();
        TR.resetMotor();
        BL.resetMotor();
        BR.resetMotor();

        // Required Stop Condition
        int success_count = TL.getSuccess_count();

        // "PID" Loop
        while((opModeIsActive() && !isStopRequested()) && (success_count < 90)){
            success_count = TL.getSuccess_count();
            // IMU PID STUFF
            imu.imu_iter(0.00000001);
            double imu_correction = imu.getCorrection();
            telemetry.addData("IMU Correction -> ", imu_correction);
            telemetry.addData("Global Angle   ->", imu.getGlobalAngle());
            if (!MotorPlus.isWithin(0.0195, imu_correction, 0.0)){
                // If correction is needed, we correct the angle
                TL.addExternalResponse(-imu_correction);
                TR.addExternalResponse(imu_correction);
                BL.addExternalResponse(-imu_correction);
                BR.addExternalResponse(imu_correction);
            } else {
                TL.addExternalResponse(0.0);
                TR.addExternalResponse(0.0);
                BL.addExternalResponse(0.0);
                BR.addExternalResponse(0.0);
            }

            // Supplying the "targets" to the Motors
            TL.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            TR.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            BL.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            BR.actuate(power_cap, target_distance, target_acceleration, target_velocity);

            // Telemetry Stuff
            telemetry.addData("TL, TR, BL, and BR Vels -> ", TL.getCurrent_velocity());
            telemetry.addData("Success Count TL -> ", TL.getSuccess_count());
            telemetry.update();
        }
    }

    public void turn(double power_cap, double target_angle, double target_acceleration,
                      double target_velocity){
        int target_distance = (int)((28/9)*target_angle);
        TL.resetMotor();
        TR.resetMotor();
        BL.resetMotor();
        BR.resetMotor();
        int success_count = TL.getSuccess_count();
        while((opModeIsActive() && !isStopRequested()) && (success_count < 90)){
            success_count = TL.getSuccess_count();
            // IMU PID STUFF
            imu.imu_iter(target_angle);
            double imu_correction = imu.getCorrection();
            telemetry.addData("IMU Correction -> ", imu_correction);
            telemetry.addData("Global Angle   ->", imu.getGlobalAngle());
            if (!MotorPlus.isWithin(0.0257, imu_correction, 0.0)){
                // If the correction is negative, we past the angle
                TL.addExternalResponse(-imu_correction);
                TR.addExternalResponse(imu_correction);
                BL.addExternalResponse(-imu_correction);
                BR.addExternalResponse(imu_correction);
            } else {
                TL.addExternalResponse(0.0);
                TR.addExternalResponse(0.0);
                BL.addExternalResponse(0.0);
                BR.addExternalResponse(0.0);
            }

            TL.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            TR.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            BL.actuate(power_cap, target_distance, target_acceleration, target_velocity);
            BR.actuate(power_cap, target_distance, target_acceleration, target_velocity);

            telemetry.addData("TL, TR, BL, and BR Vels -> ", TL.getCurrent_velocity());
            telemetry.addData("Success Count TL -> ", TL.getSuccess_count());
            telemetry.update();
        }
    }
}
