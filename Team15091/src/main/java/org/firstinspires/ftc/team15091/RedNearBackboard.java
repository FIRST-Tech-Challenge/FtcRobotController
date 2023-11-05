package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "RedNearBackboard", preselectTeleOp = "Gamepad")
public class RedNearBackboard extends AutonomousBase{
    Thread armUp = new Thread() {
        public void run() {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setTargetPosition(400);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.liftMotor.isBusy()) {
                robot.liftMotor.setPower(0.8);
                idle();
            }
            robot.liftMotor.setPower(0);
        }
    };

    Thread armDown = new Thread() {
        public void run() {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setTargetPosition(0);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.liftMotor.isBusy()) {
                robot.liftMotor.setPower(0.8);
                idle();
            }
            robot.liftMotor.setPower(0);
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();
        DistanceDetector frontDistance = new DistanceDetector((DistanceSensor)(hardwareMap.get("sensor_front")), 7, false);
        DistanceDetector frontDistanceShort = new DistanceDetector((DistanceSensor)(hardwareMap.get("sensor_front")), 5, false);
        PixelPosition initialPos = pixelDetector.objectDetected();
        robot.setArmPosition(0.7);
        if (initialPos == PixelPosition.Right) {
            robotDriver.gyroDrive(0.3d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 24.5d, -45, 3, null);
            // robotDriver.gyroDrive(0.3, 3, 45, 3, null); // move forward
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            robotDriver.gyroDrive(0.3, -10, -45, 3, null); // move backward
            armUp.run();
            robotDriver.gyroTurn(0.2, -90, 3);
            // robotDriver.gyroSlide(0.3, 10, 90, 3, null); // slide one tile to the left
            robotDriver.gyroDrive(0.3, 22.5, -90, 3, null);
            robotDriver.gyroDrive(0.2, 22.5, -90, 3, frontDistance);
            telemetry.update();
            //robotDriver.gyroDrive(0.1, 3, 90, 3, distanceToBoard);
            robotDriver.gyroSlide(0.1, -12, 90, 3, null);
            robot.armServo.setPosition(0);
            sleep(1500);
            robot.setBowlPosition(0.45);
            sleep(1500);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, -17.5, 90, 3, null);
            robotDriver.gyroDrive(0.3, 22.5, 90,  3, frontDistance);
        }
        else if (initialPos == PixelPosition.Left) {
            robotDriver.gyroDrive(0.3d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 25d, 45, 3, null);
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            robotDriver.gyroDrive(0.2d, -25d, 0, 3, null);
            robotDriver.gyroDrive(0.3d, -12d, 0, 3, null);
            robotDriver.gyroSlide(0.2d, -45, 0, 5, null);
            /*
            robotDriver.gyroDrive(0.3, -10, 45, 3, null); // move backward
            armUp.run();
            robotDriver.gyroTurn(0.2, -90, 3);
            robotDriver.gyroDrive(0.3, 22.5, 90, 3, null);
            robotDriver.gyroDrive(0.2, 22.5, 90, 3, frontDistance);
            robotDriver.gyroSlide(0.2, -15,90, 5, null);
            robot.armServo.setPosition(0);
            sleep(1500);
            robot.setBowlPosition(0.45);
            sleep(1500);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, 34, 90, 5, null);
            robotDriver.gyroDrive(0.3, 22.5, 90,  5, frontDistance);
            */
        }
        else { // pixel in the middle position
            robotDriver.gyroDrive(0.2, 29.5, 0, 5, null);
            robotDriver.gyroDrive(0.2, -7, 0, 5, null);
            armUp.run();
            robotDriver.gyroTurn(0.1, -90, 5);
            robotDriver.gyroDrive(0.3, 22.5, 90, 5, null);
            robotDriver.gyroDrive(0.2, 22.5, 90, 5, frontDistance);
            //robotDriver.gyroSlide(0.2, -10, 90, 5, null);
            robot.armServo.setPosition(0);
            sleep(1500);
            robot.setBowlPosition(0.45);
            sleep(1500);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, -29, 90, 5, null);
            robotDriver.gyroDrive(0.3, 22.5, 90,  5, frontDistance);
        }
    }
}
