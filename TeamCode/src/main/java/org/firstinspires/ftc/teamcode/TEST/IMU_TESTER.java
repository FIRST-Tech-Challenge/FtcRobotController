package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMap.HMap;
import org.firstinspires.ftc.teamcode.HardwareMap.IMUPlus;
import org.firstinspires.ftc.teamcode.HardwareMap.MotorPP;

class IMU_TESTER extends LinearOpMode {

    HMap robot = new HMap();
    IMUPlus imu;


    double[] dPid_ = new double[]{};
    double[] aPid_ = new double[]{};
    double[] vPid_ = new double[]{};

    MotorPP TL, TR, BL, BR;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
    }
}
