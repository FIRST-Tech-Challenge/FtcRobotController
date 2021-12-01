package org.firstinspires.ftc.team8923_2021;

public class BlueStorageUnit extends MasterAutonomous {
    public void runOpMode() {
        while (opModeIsActive()) {
            double referenceAngle = imu.getAngularOrientation().firstAngle;
            moveForward(30, 10, 10);
            imuPivot(referenceAngle, -90, 35, 0.015, 3.0);
            moveForward(5, 10, 10);
        }
    }
}