package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Park in blue Box")
public class ParkInBlueBox extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {

        robot = new RobotClass(hardwareMap,telemetry, this);

        waitForStart();

//        robot.parkBlue();

//        int seeRed = 0;
//        String direction = null;
//        while (direction != "middle") {
//
//            int loopTimes = 0;
//
//            if (direction == "left") {
//                robot.strafeLeft(0.1,0.1);
//            } else if (direction == "right"){
//                robot.strafeRight(0.1, 0.1);
//            }
//            if (robot.colorSensorLeft.red() > 120) {
//                seeRed = 1;
//                while (robot.colorSensorRight.red() < 120) {
//                    robot.strafeLeft(0.1, 0.1);
//                }
//                robot.strafeLeft(0.1, 0.1);
//                if (robot.colorSensorLeft.red() > 120) {
//                    robot.strafeRight(0.1,0.1);
//                } else {
//                    while (seeRed < 3) {
//                        robot.strafeRight(0.1,0.1);
//                        if (robot.colorSensorRight.red() > 120) {
//                            seeRed++;
//                        }
//                    }
//                    robot.strafeLeft(0.1,0.1);
//                }
//                direction = "middle";
//            }
//
//            if (robot.colorSensorRight.red() > 120) {
//                seeRed = 1;
//                robot.strafeLeft(0.1,0.1);
//                if (robot.colorSensorLeft.red() > 120) {
//                    robot.strafeRight(0.1,0.1);
//                } else {
//                    while (seeRed < 3) {
//
//                        if (robot.colorSensorRight.red() > 120) {
//                            seeRed++;
//                        }
//                    }
//                    robot.strafeLeft(0.1,0.1);
//                }
//                direction = "middle";
//            }
//
//            if (robot.colorSensorMiddle.red() > 120) {
//                seeRed = 1;
//                while (seeRed < 2) {
//                    robot.strafeLeft(0.1,0.1);
//                    if (robot.colorSensorLeft.red() > 120) {
//                        seeRed++;
//                    }
//                    if (robot.colorSensorRight.red() > 120) {
//                        seeRed++;
//                    }
//                }
//                seeRed = 0;
//            }
//
//            if (seeRed == 0) {
//                direction = "left";
//            }
//
//            loopTimes++;
//
//            if (loopTimes >= 10) {
//                direction = "right";
//                seeRed = 1;
//            }
//
//        }

    }

}
