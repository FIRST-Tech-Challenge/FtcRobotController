package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCarouselNoIntake")
public class RedCarouselNoIntake extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();

        while (opModeIsActive()) {
            double referenceAngle = imu.getAngularOrientation().firstAngle;

            imuPivot(referenceAngle,15, 35, 0.015, 3.0);
            moveForward(-3.5,10,10);
            spinCarouselRed();
            moveForward(2.5, 10, 10);
            imuPivot(referenceAngle, -10, 30, 0.015, 3.0);
            moveForward(90, 10, 10);
            break;
        }
    }
}