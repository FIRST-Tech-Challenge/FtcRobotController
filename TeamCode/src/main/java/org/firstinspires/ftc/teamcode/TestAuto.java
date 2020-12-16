package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Diff Swerve Test Auto", group = "Linear Opmode")

public class TestAuto extends LinearOpMode {
    Robot robot;
    public boolean willResetIMU = true;
    double cmDistance = 2;
    double speed = 0.5;
    Vector2d direction = new Vector2d(0,1);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.initIMU();

        waitForStart();
        long startTime = System.currentTimeMillis();
        long subtractTime = 0;
        //simple sequence to demonstrate the three main autonomous primitives

        //rotate modules to face to the right
       // robot.driveController.rotateModules(Vector2d.RIGHT, false,1000, this);

        //drive 20 cm to the right (while facing forward)
        //robot.driveController.drive(Vector2d.RIGHT, 200, .5, this);



        //robot.driveController.drive(Vector2d.RIGHT, 200, 1, this);
        //robot.driveController.updateUsingJoysticks(Vector2d.RIGHT.scale(Math.sqrt(2)), new Vector2d(0,0), false);
        //robot.driveController.updateAbsRotation(Vector2d.RIGHT.scale(Math.sqrt(2)), new Vector2d(0,0).scale(Math.sqrt(2)), 0.7);

        robot.driveController.resetDistanceTraveled();
        while (robot.driveController.getDistanceTraveled() < cmDistance && opModeIsActive()) {
            //slows down drive power in certain range
            robot.driveController.updateTracking();
            robot.driveController.update(direction.normalize(speed), 0);

            telemetry.addData("Driving robot", "");
            telemetry.addData("Right Orientation", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
            telemetry.addData("Distance", robot.driveController.moduleRight.getDistanceTraveled());
            telemetry.addData("Encoder", robot.driveController.moduleRight.motor1.getCurrentPosition());
            telemetry.update();
        }
        robot.driveController.update(Vector2d.ZERO, 0);


        //robot.driveController.moduleRight.updateTarget(Vector2d.BACKWARD, 0);
        //robot.driveController.moduleLeft.updateTarget(Vector2d.BACKWARD, 0);
        //sleep(2000);


        //turn to face robot right
        //robot.driveController.rotateRobot(Angle.RIGHT, this);
    }

    private Angle getCurrentOrientation() {
        double rawAngle = (double)(robot.driveController.moduleLeft.motor1.getCurrentPosition() + robot.driveController.moduleLeft.motor2.getCurrentPosition()/2.0);//motor2-motor1 makes ccw positive (?)
        return new Angle(rawAngle, Angle.AngleType.ZERO_TO_360_HEADING);
    }
}
