package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCarouselNoIntake")
public class RedCarouselWarehouse extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();

        while (opModeIsActive()) {
            double referenceAngle = imu.getAngularOrientation().firstAngle;

            imuPivot(referenceAngle,30, 35, 0.015, 3.0);
            moveForward(-3.9,10,10);
            spinCarouselRed();
            moveForward(2.5, 10, 10);
            imuPivot(referenceAngle, 15, 35, 0.015, 3.0);
            moveForward(98, 10, 10);
            break;
        }
    }
}