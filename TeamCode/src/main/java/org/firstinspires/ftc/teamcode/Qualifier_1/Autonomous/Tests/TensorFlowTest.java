/**
 * Tests Tensor Flow. If Tensor Flow detects
 * 4 rings, the robot should move forward.
 * If it detects 1 ring, the robot should
 * move left. Otherwise, the robot should move
 * right.
 *
 * @author  Aamod
 * @version 1.0
 * @since   2020-November-5
 * @status: Not fully working
 */

package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Chassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous
//@Disabled
public class TensorFlowTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        int rings = -1;
        int i = 0;
        robot.initTensorFlow();
        while (runtime.seconds()<4) {
            robot.runTensorFlow();
            sleep(50);
            rings = robot.tensorFlow.getNumberOfRings();
            telemetry.addData("Number of Rings: ", "i=%d %d", i++, rings);
        }
        telemetry.update();
        sleep(2000);
        //robot.stopTensorFlow();

        telemetry.addData("FinalNumOfRings: ", robot.tensorFlow.getNumberOfRings());
        telemetry.update();

        waitForStart();
        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopTensorFlow();

        robot.moveBackward(58, 0.8);

        if (rings == 4) {
            robot.moveBackward(6, 0.5);
            sleep(200);
            robot.turnInPlace(-6, 0.6);
            sleep(200);
            robot.moveBackward(70, 0.8);
            sleep(200);
            robot.moveBackward(28, 0.5);
            sleep(200);
            robot.moveWobbleGoalServo(true);
            sleep(500);
            robot.turnInPlace(8, 0.6);
            robot.moveForward(37, 0.8);
            telemetry.addData("NumberOfRings: ", 4);
            telemetry.update();
            sleep(2000);
        } else if (rings == 1) {
            robot.moveBackward(6, 0.5);
            sleep(200);
            robot.turnInPlace(4,0.8);
            sleep(200);
            robot.moveBackward(80,0.8);
            sleep(200);
            robot.moveWobbleGoalServo(true);
            sleep(1000);
            robot.moveForward(8,0.8);
            telemetry.addData("NumberOfRings: ", 1);
            telemetry.update();
            sleep(2000);
        } else {
            robot.moveBackward(6, 0.5);
            sleep(200);
            robot.turnInPlace(-20, 0.6);
            sleep(200);
            robot.moveForward(6, 0.8);
            robot.moveWobbleGoalServo(true);
            sleep(1000);
            telemetry.addData("NumberOfRings: ", 0);
            telemetry.update();
            sleep(2000);
        }
    }
}
