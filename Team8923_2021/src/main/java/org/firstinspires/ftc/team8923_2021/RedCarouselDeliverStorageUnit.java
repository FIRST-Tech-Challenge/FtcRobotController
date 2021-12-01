package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedCarouselDeliverStorageUnit")
public class RedCarouselDeliverStorageUnit extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();

        while (opModeIsActive()) {
            double referenceAngle = imu.getAngularOrientation().firstAngle;
            moveForward(-2.8, 10, 10);
            spinCarouselBlue();
        }
    }
}