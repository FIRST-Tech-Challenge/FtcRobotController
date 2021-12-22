package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueCarouselWarehouse")
public class BlueCarouselWarehouse extends MasterAutonomous{
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        double referenceAngle =  imu.getAngularOrientation().firstAngle;
        moveForward(-2.8, 10, 10);
        spinCarouselBlue();
        moveForward(13, 50, 10);
        imuPivot(referenceAngle, -90, 30, 0.015, 3.0);
        moveForward(15, 50, 10);
        imuPivot(referenceAngle, referenceAngle, 30, 0.015, 3.0);
        moveForward(80, 50, 10);
    }
}

