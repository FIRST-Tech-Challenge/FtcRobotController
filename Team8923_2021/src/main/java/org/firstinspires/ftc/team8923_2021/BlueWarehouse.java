package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueWarehouseNoIntake")
public class BlueWarehouse extends MasterAutonomous{
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        //moveForward(16, 80, 10);
        double referenceAngle =  imu.getAngularOrientation().firstAngle; // Get a reference angle from the IMU for future movements using IMU
        imuPivot(referenceAngle, 90, 30, 0.015, 3.0);
        moveForward(50, 80, 10);
    }
}
