package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCarouselNoIntake")
public class RedCarouselNoIntake extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();

        while (opModeIsActive()) {
            double referenceAngle = imu.getAngularOrientation().firstAngle;
            moveForward(-2.5, 10, 10);
            moveForward(13, 50, 10);
            spinCarouselRed();
            imuPivot(referenceAngle, -90, 30, 0.015, 3.0);
            moveForward(10, 50, 10); //check distance on this
            imuPivot(referenceAngle, referenceAngle, 30, 0.015, 3.0);
            moveForward(80, 50, 10);
            break;
        }
    }
}