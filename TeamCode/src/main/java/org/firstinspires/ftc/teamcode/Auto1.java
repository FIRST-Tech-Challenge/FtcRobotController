package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.DistanceSensorBot;

@Autonomous(name="Auto 1", group="Autos")
@Disabled
public class Auto1 extends LinearOpMode {

    protected DistanceSensorBot robot = new DistanceSensorBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.isRepeating = true;
        waitForStart();
        int[] pos;
        pos = new int[]{0, 0};
        if (pos[1] == 0) {
            robot.servoPosIndex = 1;
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 3, 0.6, false, 0, true);
            robot.goToInOutPosition(1);
            robot.setArmPositionNoWait(-500, 0.3);
            //robot.servoPosIndex = 1;
            robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
            robot.goBacktoStartAngle();
            robot.sleep(800);
            if (pos[0] == 0) {
                robot.setArmPositionNoWait(-910, 0.18);
                robot.sleep(1300);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 4, 0.4);
                robot.sleep(400);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(600);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.setArmPositionNoWait(-25, 0.1);
                //robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 2, 0.9);
            } else if (pos[0] == 1) {
                robot.setArmPositionNoWait(-760, 0.18);
                robot.sleep(600);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 8, 0.4);
                robot.sleep(200);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(600);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.setArmPositionNoWait(-25, 0.1);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 4, 0.9);
            } else {
                robot.setArmPositionNoWait(-580, 0.18);
                robot.sleep(400);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 9, 0.8);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(600);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.setArmPositionNoWait(-25, 0.1);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 5, 0.9);
            }
            robot.goToAnglePID(-50);
            //robot.goToAngle(50, 0.1);
            robot.goToInOutPosition(0);
            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 10, 0.6, true, -50, false);
            //robot.sleep(200, "after gyro wait");
            robot.goToInOutPosition(1);
            robot.goToAnglePID(-90);
            robot.goToAnglePID(-90);
            //robot.goToAngle(90, 0.1);
            robot.driveStraightByTime(robot.DIRECTION_LEFT, 500, 0.4);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 1, 0.4);

            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 18, 0.6, true, -89, true);
            robot.sleep(200);
            robot.goToAnglePID(-90);
            robot.sleep(1000);
            robot.autoGrabFreight(0.12);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 14, 0.6, true, -89, true);
            robot.goToInOutPosition(0);
            robot.goToAnglePID(-30);

            //robot.goToAngle(30, 0.1);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 6.5, 0.6, true, -30, false);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 10, 0.2, true, -30, true);
            robot.sleep(200, "after gyro wait");
            //robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 1, 0.3, true, 25, true);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 2, 0.2, true, -30, true);
            robot.sleep(200);
            robot.goToAnglePID(-25);
            robot.sleep(300);
            //robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 0.5, 0.18, true, 30, true);
            robot.sleep(300);
            //robot.goToAnglePID(30);
            robot.servoPosIndex = 0;
            robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
            robot.sleep(300);
            robot.servoPosIndex = 2;
            robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
            robot.goToInOutPosition(1);
            robot.setArmPositionNoWait(-25, 0.1);

            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 8, 0.6, true, -30, false);
            robot.goToInOutPosition(0);
            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 11, 0.4, true, -30, true);
            robot.sleep(200, "after gyro wait");
            robot.goToInOutPosition(1);
            robot.goToAnglePID(-90);
            robot.driveStraightByTime(robot.DIRECTION_LEFT, 600, 0.4);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 1, 0.4);

            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 20, 0.8, true, -89, true);
            robot.sleep(200);
        } else {
            robot.servoPosIndex = 1;
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 3, 0.6, false, 0, true);
            robot.goToInOutPosition(1);
            robot.setArmPositionNoWait(-500, 0.3);
            //robot.servoPosIndex = 1;
            robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
            robot.goBacktoStartAngle();
            robot.sleep(800);
            if (pos[0] == 0) {
                robot.setArmPositionNoWait(-910, 0.18);
                robot.sleep(1300);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 4, 0.4);
                robot.sleep(400);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(600);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.setArmPositionNoWait(-25, 0.1);
                //robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 2, 0.9);
            } else if (pos[0] == 1) {
                robot.setArmPositionNoWait(-760, 0.18);
                robot.sleep(600);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 8, 0.4);
                robot.sleep(200);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(600);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.setArmPositionNoWait(-25, 0.1);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 4, 0.9);
            } else {
                robot.setArmPositionNoWait(-580, 0.18);
                robot.sleep(400);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 9, 0.8);
                robot.servoPosIndex = 0;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.sleep(600);
                robot.servoPosIndex = 2;
                robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
                robot.setArmPositionNoWait(-25, 0.1);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 5, 0.9);
            }
            robot.goToAnglePID(50);
            //robot.goToAngle(50, 0.1);
            robot.goToInOutPosition(0);
            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 10, 0.6, true, 50, false);
            //robot.sleep(200, "after gyro wait");
            robot.goToInOutPosition(1);
            robot.goToAnglePID(90);
            robot.goToAnglePID(90);
            //robot.goToAngle(90, 0.1);
            robot.driveStraightByTime(robot.DIRECTION_RIGHT, 500, 0.4);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 1, 0.4);

            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 15, 0.6, true, 89, true);
            robot.sleep(200);
            robot.goToAnglePID(90);
            robot.sleep(1000);
            robot.autoGrabFreight(0.12);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 16, 0.6, true, 89, true);
            robot.goToInOutPosition(0);
            robot.goToAnglePID(30);

            //robot.goToAngle(30, 0.1);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 4.5, 0.6, true, 30, false);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 9, 0.2, true, 30, true);
            robot.sleep(200, "after gyro wait");
            //robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 1, 0.3, true, 25, true);
            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 2, 0.2, true, 30, true);
            robot.sleep(200);
            robot.goToAnglePID(25);
            robot.sleep(300);
            //robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 0.5, 0.18, true, 30, true);
            robot.sleep(300);
            //robot.goToAnglePID(30);
            robot.servoPosIndex = 0;
            robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
            robot.sleep(300);
            robot.servoPosIndex = 2;
            robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
            robot.goToInOutPosition(1);
            robot.setArmPositionNoWait(-25, 0.1);

            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 6, 0.6, true, 30, false);
            robot.goToInOutPosition(0);
            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 11, 0.4, true, 30, true);
            robot.sleep(200, "after gyro wait");
            robot.goToInOutPosition(1);
            robot.goToAnglePID(90);
            robot.driveStraightByTime(robot.DIRECTION_RIGHT, 600, 0.4);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 1, 0.4);

            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 20, 0.8, true, 89, true);
            robot.sleep(200);
//            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 19, 0.8, true, 88, true);
//            robot.sleep(200);
//            robot.goToAnglePID(90);
//            robot.autoGrabFreight(0.12);
//            robot.controlCoreHex(1, 0);
//            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 17, 0.7, true, 88, true);
//            robot.goToInOutPosition(0);
//            robot.goToAnglePID(22);
//
//            //robot.goToAngle(30, 0.1);
//            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 3.5, 0.9, true, 28, false);
//            robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 4, 0.2, true, 28, false);
//            robot.controlCoreHex(0, 0);
//            robot.sleep(200, "after gyro wait");
//            //robot.driveStraightByGyro(robot.DIRECTION_BACKWARD, 1, 0.3, true, 25, true);
//            robot.goToAnglePID(28);
//            robot.sleep(300);
//            //robot.goToAnglePID(28);
//            robot.servoPosIndex = 0;
//            robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
//            robot.sleep(300);
//            robot.servoPosIndex = 2;
//            robot.wobblePinch.setPosition(robot.servoPositions[robot.servoPosIndex]);
//            robot.goToInOutPosition(1);
//            robot.setArmPositionNoWait(-25, 0.1);
//            robot.goToInOutPosition(0);
//
//            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 10, 0.8, true, 35, false);
//            robot.goToInOutPosition(1);
//            //robot.sleep(200, "after gyro wait");
//            robot.goToAnglePID(90);
//            robot.driveStraightByTime(robot.DIRECTION_RIGHT, 500, 0.4);
//            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 1, 0.4);
//
//            robot.driveStraightByGyro(robot.DIRECTION_FORWARD, 15, 0.8, true, 89, true);
//            robot.controlCoreHex(0, 1);
//            robot.sleep(2000);
        }
    }
}
