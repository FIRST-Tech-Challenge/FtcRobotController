package org.firstinspires.ftc.team8923_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="CompetitionAuto")
public class CompetitionAuto extends MasterAutonomous {
    @Override
    public void runOpMode() throws InterruptedException{
        //configureAutonomous();
        initAuto();
        double referenceAngle = imu.getAngularOrientation().firstAngle;
        telemetry.clear();
        telemetry.update();

        waitForStart();
        //telemetry.clear();


        while(opModeIsActive()){
            double refAngle =  imu.getAngularOrientation().firstAngle; // Get a reference ange from the IMU for future movements using IMU
            sleep(500);
            moveAuto(0, 61, 0.5, 0.3);
            moveAuto(12, 0, 0.5, 0.3);
            imuPivot(refAngle, 1, 0.3, 0.015, 3.0);
            //shootRings();

            motorLift.setTargetPosition(870);
            motorLift.setPower(Math.max((motorLift.getTargetPosition() - motorLift.getCurrentPosition()) * (1 / 75.0), 1.0));
            motorShooter.setPower(-0.33);
            sleep(3000);
            servoFlicker.setPosition(0.4);
            sleep(100);
            servoFlicker.setPosition(0.55);
            sleep(500);
            servoFlicker.setPosition(0.4);
            sleep(100);
            servoFlicker.setPosition(0.55);
            sleep(600);
            servoFlicker.setPosition(0.4);
            sleep(100);
            servoFlicker.setPosition(0.55);
            sleep(500);
            motorShooter.setPower(0.0);
            motorLift.setTargetPosition(0);
            motorLift.setPower(Math.max((motorLift.getTargetPosition() - motorLift.getCurrentPosition()) * (1 / 75.0), 1.0));
            sleep(1500);
            moveAuto(0, 12, 0.5, 0.3);
            break;

        }
    }

}
