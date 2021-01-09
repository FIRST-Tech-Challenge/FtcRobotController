package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.SLAM;

@Autonomous(name = "Diff Swerve Test Auto", group = "Linear Opmode")

public class TestAuto extends LinearOpMode {
    Robot robot;
    public boolean willResetIMU = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.initIMU();

        waitForStart();
        long startTime = System.currentTimeMillis();
        long subtractTime = 0;
        //simple sequence to demonstrate the three main autonomous primitives

        //rotate modules to face to the right
        //robot.driveController.rotateModules(Vector2d.RIGHT, false, 10000, this);

        //drive 20 cm to the right (while facing forward)
        //robot.driveController.drive(Vector2d.FORWARD, 100 , 1, this);
        //robot.driveController.rotateRobot(Angle.RIGHT, this);
        robot.driveController.rotateModules(Vector2d.FORWARD, false, 2000, this);
        sleep(500);
        robot.driveController.rotateModules(Vector2d.LEFT, false, 2000, this);
        sleep(500);
        robot.driveController.rotateModules(Vector2d.BACKWARD, false, 2000, this);
        sleep(500);
        robot.driveController.rotateModules(Vector2d.RIGHT, false, 2000, this);
        sleep(500);
        robot.driveController.rotateModules(Vector2d.FORWARD, false, 2000, this);
        robot.driveController.drive(Vector2d.FORWARD, 50 , .5, this);
        robot.driveController.drive(Vector2d.UNIT_CIRCLE_60, 50 , .5, this);
        robot.driveController.drive(Vector2d.RIGHT, 50 , .5, this);
        robot.driveController.drive(Vector2d.UNIT_CIRCLE_120, 50 , .5, this);
        robot.driveController.rotateModules(Vector2d.FORWARD, false, 2000, this);

        //robot.driveController.drive(Vector2d.RIGHT, 200, 1, this);
        //robot.driveController.updateUsingJoysticks(Vector2d.RIGHT.scale(Math.sqrt(2)), new Vector2d(0,0), false);
        //robot.driveController.updateAbsRotation(Vector2d.RIGHT.scale(Math.sqrt(2)), new Vector2d(0,0).scale(Math.sqrt(2)), 0.7);

//        robot.driveController.resetDistanceTraveled();
//        while (robot.driveController.getDistanceTraveled() < cmDistance && opModeIsActive()) {
//            //slows down drive power in certain range
//            robot.driveController.updateTracking();
//            robot.driveController.update(direction.normalize(speed), 0);
//
//            telemetry.addData("Driving robot", "");
//            telemetry.addData("Right Orientation", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
//            telemetry.addData("Distance", robot.driveController.moduleRight.getDistanceTraveled());
//            telemetry.addData("Encoder", robot.driveController.moduleRight.motor1.getCurrentPosition());
//            telemetry.update();
//        }
//        robot.driveController.update(Vector2d.ZERO, 0);


        //robot.driveController.moduleRight.updateTarget(Vector2d.BACKWARD, 0);
        //robot.driveController.moduleLeft.updateTarget(Vector2d.BACKWARD, 0);
        //sleep(2000);


        //turn to face robot right
        //robot.driveController.rotateRobot(Angle.RIGHT, this);
    }

}
