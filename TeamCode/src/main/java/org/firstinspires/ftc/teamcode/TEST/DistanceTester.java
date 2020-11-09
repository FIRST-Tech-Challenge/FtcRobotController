package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMap.HMap;
import org.firstinspires.ftc.teamcode.HardwareMap.IMUPlus;
import org.firstinspires.ftc.teamcode.HardwareMap.MotorPP;
import org.firstinspires.ftc.teamcode.HardwareMap.MotorPlus;

@Autonomous(name="MotorPP Distance(0.40)")
public class DistanceTester extends LinearOpMode {
    HMap robot = new HMap();
    IMUPlus imu;

    double[] dPid_ = new double[]{2.832556, 0.0000098546, 0.00008}; // distance pid values
    double[] aPid_ = new double[]{0.000000504, 0.00000000002998311, 0.000000000000702983749}; // acceleration pid values
    double[] vPid_ = new double[]{.0000274, 0.0000000004643, 0}; // velocity pid values
    double[] imuPid_ = new double[]{0.000002246, 0.000000000000029373, 0.0000000000000000001024}; // imu based pid values

    MotorPP TL, TR, BL, BR;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        imu = new IMUPlus(robot.imu, imuPid_);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Done...");
        telemetry.update();

        TL = new MotorPP(new DcMotor[]{robot.TL}, dPid_, aPid_, vPid_);
        TR = new MotorPP(new DcMotor[]{robot.TR}, dPid_, aPid_, vPid_);
        BL = new MotorPP(new DcMotor[]{robot.BL}, dPid_, aPid_, vPid_);
        BR = new MotorPP(new DcMotor[]{robot.BR}, dPid_, aPid_, vPid_);

        waitForStart();
        imu.resetAngle();

        // Actual Driving + Moving Code Goes here!
        drive(1.0, 0.40, 0.0000124, .114);
    }

    public void drive(double power_cap, double target_distance, double target_acceleration,
                      double target_velocity){
        TL.resetMotor();
        TR.resetMotor();
        BL.resetMotor();
        BR.resetMotor();
        while((opModeIsActive() && !isStopRequested()) && (TL.getSuccess_count() < 90 ||
                TR.getSuccess_count() < 90)){

            // IMU PID STUFF
            imu.imu_iter(0.0001);
            double imu_correction = imu.getCorrection();
            telemetry.addData("IMU Correction -> ", imu_correction);
            telemetry.addData("Global Angle   ->", imu.getGlobalAngle());
            if (!MotorPlus.isWithin(0.0257, imu_correction, 0.0)){
                // If the correction is negative, we past the angle
                TL.addExternalResponse(imu_correction);
                TR.addExternalResponse(-imu_correction);
                BL.addExternalResponse(imu_correction);
                BR.addExternalResponse(-imu_correction);
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
