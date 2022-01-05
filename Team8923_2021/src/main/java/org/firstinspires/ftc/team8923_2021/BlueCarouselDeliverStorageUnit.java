package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueCarouselDeliverStorageUnit")
public class BlueCarouselDeliverStorageUnit extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        double referenceAngle = imu.getAngularOrientation().firstAngle;
        moveForward(-2.8, 10, 10);
        spinCarouselBlue();
        moveForward(30, 10, 10);
        imuPivot(referenceAngle,-90, 35, 0.015, 3.0);
        moveForward(5.0, 10, 10);
        imuPivot(referenceAngle, 90, 35, 0.015, 3.0);
        moveForward(-2.0, 10, 10);
        autoDeliver();
        imuPivot(referenceAngle, 90, 35, 0.015, 3.0);
        moveForward(30, 10, 10);
    }
}