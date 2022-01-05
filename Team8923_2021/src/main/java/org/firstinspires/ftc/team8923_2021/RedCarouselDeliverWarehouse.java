package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCarouselDeliverWarehouse")
public class RedCarouselDeliverWarehouse extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        double referenceAngle = imu.getAngularOrientation().firstAngle;
        imuPivot(referenceAngle, 30, 35, 0.015, 3.0);
        moveForward(-3.9, 10, 10);
        spinCarouselRed();
        moveForward(30.0, 10, 10);
        imuPivot(referenceAngle, 235, 35, 0.015, 3.0);
        moveForward(-11, 10, 10);
        autoDeliver();
        moveForward(2, 10, 10);
        imuPivot(referenceAngle,8,35, 0.015, 3.0);
        moveForward(30.0, 10, 10);
    }
}