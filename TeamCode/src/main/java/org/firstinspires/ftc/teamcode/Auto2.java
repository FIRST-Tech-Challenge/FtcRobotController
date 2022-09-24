package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.DistanceSensorBot;

@Autonomous(name="Auto 2", group="Autos")
@Disabled
public class Auto2 extends LinearOpMode {

    protected DistanceSensorBot robot = new DistanceSensorBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.isRepeating = true;
        waitForStart();
        int[] pos;
        pos = new int[]{0, 0};
        if (pos[1] == 0) {
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 10.5, 0.5);
            robot.goToAnglePID(90);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 9, 0.2);
            robot.driveStraightByTime(robot.DIRECTION_RIGHT, 1500, 0.2);
            robot.driveStraightByTime(robot.DIRECTION_BACKWARD, 500, 0.2);
            //robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 2, 0.2);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 0.2, 0.14);
            //robot.toggleSpinner(0.35, false);
            robot.sleep(3500);
            //robot.toggleSpinner(0.35, false);

            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 5, 0.6);
            robot.sleep(200);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 19, 0.8);
            robot.sleep(500);
            robot.goToAnglePID(0);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 6, 0.6, true, 0, true);
            robot.goToAnglePID(0);
            robot.controlExtension(true);
            robot.sleep(500);
            if (pos[0] == 0) {
                robot.setArmPositionNoWait(-935, 0.2);
                robot.sleep(2000);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 6, 0.2);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(300);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 2, 0.5);
            } else if (pos[0] == 1) {
                robot.setArmPositionNoWait(-760, 0.18);
                robot.sleep(2000);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 6, 0.2);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(300);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 2, 0.5);
            } else {
                robot.setArmPositionNoWait(-600, 0.18);
                robot.sleep(2000);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 15, 0.2);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(300);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 11, 0.7);
            }
            robot.setArmPositionNoWait(-25, 0.1);
            robot.goToAnglePID(170);
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 18, 0.6);
            robot.goToAnglePID(170);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 5, 0.3);
            //robot.driveStraightByDistance(robot.DIRECTION_LEFT, 3, 0.4);
            robot.goToInOutPosition(0);
        } else {
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 6.5, 0.5);
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 9, 0.2);
            robot.driveStraightByTime(robot.DIRECTION_RIGHT, 500, 0.2);
            //robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 2, 0.2);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 0.2, 0.14);
            //robot.toggleSpinner(0.35, true);
            robot.sleep(3500);
            //robot.toggleSpinner(0.35, true);

            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 5, 0.6, true, 0, true);
            robot.sleep(200);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 28, 0.8);
            robot.sleep(500);
            robot.goToAnglePID(170);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 6, 0.6, true, 170, true);
            robot.goToAnglePID(170);
            robot.controlExtension(true);
            robot.sleep(500);
            if (pos[0] == 0) {
                robot.setArmPositionNoWait(-935, 0.2);
                robot.sleep(2000);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 6, 0.2);
                robot.sleep(500);
                robot.goToAnglePID(175);
                robot.sleep(500);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(300);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 2, 0.5);
            } else if (pos[0] == 1) {
                robot.setArmPositionNoWait(-760, 0.18);
                robot.sleep(2000);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 7, 0.2);
                robot.sleep(500);
                robot.goToAnglePID(175);
                robot.sleep(500);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(300);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 2, 0.5);
            } else {
                robot.setArmPositionNoWait(-600, 0.18);
                robot.sleep(2000);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 15, 0.2);
                robot.sleep(500);
                robot.goToAnglePID(175);
                robot.sleep(500);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(300);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 11, 0.7);
            }
            robot.setArmPositionNoWait(-25, 0.1);
            robot.goToAnglePID(175);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 16, 0.4);
            //robot.driveStraightByDistance(robot.DIRECTION_LEFT, 3, 0.4);
            robot.sleep(1000);
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 3, 0.4);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 17, 0.4);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD,3,0.4);
            robot.goToInOutPosition(0);
        }
    }
}
