package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMap.HMap;
import org.firstinspires.ftc.teamcode.HardwareMap.IMUPlus;
import org.firstinspires.ftc.teamcode.HardwareMap.MotorPP;
import org.firstinspires.ftc.teamcode.HardwareMap.MotorPlus;

import static org.firstinspires.ftc.teamcode.MISC.CONSTANTS.aPid_;
import static org.firstinspires.ftc.teamcode.MISC.CONSTANTS.dPid_;
import static org.firstinspires.ftc.teamcode.MISC.CONSTANTS.imuPid_;
import static org.firstinspires.ftc.teamcode.MISC.CONSTANTS.vPid_;

@Autonomous(name = "IMU PID Tester")
public class IMU_TESTER extends LinearOpMode {
    HMap robot = new HMap();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Done...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            // IMU PID STUFF
            robot.imu.imu_iter(0.0001);
            double imu_correction = robot.imu.getCorrection();
            telemetry.addData("IMU Correction -> ", imu_correction);
            telemetry.addData("Global Angle   ->", robot.imu.getGlobalAngle());
            telemetry.update();
            if (!MotorPlus.isWithin(0.03, imu_correction, 0.0)){
                // If the correction is negative, we past the angle
                robot.TL.setPower(-imu_correction);
                robot.TR.setPower(imu_correction);
                robot.BL.setPower(-imu_correction);
                robot.BR.setPower(imu_correction);
            } else {
                robot.TL.setPower(0);
                robot.TR.setPower(0);
                robot.BL.setPower(0);
                robot.BR.setPower(0);
            }
        }
    }
}
