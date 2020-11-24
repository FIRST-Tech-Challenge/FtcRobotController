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

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Chassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous
//@Disabled
public class TensorFlowTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        ElapsedTime runtime = new ElapsedTime();
        Chassis chassis = new Chassis(this);

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

        if (rings == 4) {
            chassis.moveForward(6, 0.5);
            sleep(200);
            chassis.turnInPlace(14, 0.6);
            sleep(200);
            chassis.moveForward(109, 0.8);
            sleep(200);
            chassis.turnInPlace(-14, 0.6);
            chassis.moveForward(-44, 0.8);
            telemetry.addData("NumberOfRings: ", 4);
            telemetry.update();
            sleep(2000);
        } else if (rings == 1) {
            chassis.turnInPlace(-2,0.8);
            sleep(200);
            chassis.moveForward(80,0.8);
            sleep(200);
            chassis.moveForward(-8,0.8);
            telemetry.addData("NumberOfRings: ", 1);
            telemetry.update();
            sleep(2000);
        } else {
            chassis.moveForward(6, 0.5);
            sleep(200);
            chassis.turnInPlace(20, 0.6);
            sleep(200);
            chassis.moveForward(65, 0.8);
            telemetry.addData("NumberOfRings: ", 0);
            telemetry.update();
            sleep(2000);
        }
    }
}
