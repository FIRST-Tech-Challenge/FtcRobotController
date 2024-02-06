package org.firstinspires.ftc.teamcode.compautoG2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.GlobalConfig;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_POS;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.MotionHardwareG2;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;

import Auto.Left;

@Config
@Autonomous
public class AutoRRG2 extends LinearOpMode {
    public ALLIANCE_POS alliancePos = ALLIANCE_POS.RIGHT;

    public GlobalConfig globalConfigG2 = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);
    MotionHardwareG2 robot = new MotionHardwareG2(this, globalConfigG2);
    VisionHardware vision = new VisionHardware(this, alliancePos);
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init();
        vision.init();

        waitForStart();

        while (opModeIsActive()) {
            PropPosition propPosition = vision.detectProp();

            switch (propPosition) {
                case MIDDLE:
                    robot.moveRobot(.5,40, 50);
                    robot.dropperUp();
                    robot.turnRobot(MotionHardwareG2.Direction.RIGHT, 9, .5, 10);
                    robot.moveRobot(.5, 30, 50);
                    break;
                case RIGHT:
                    robot.moveRobot(.5,40, 50);
                    robot.turnRobot(MotionHardwareG2.Direction.LEFT, 9, .5, 10);
                    robot.dropperUp();
                    robot.moveRobot(.5, 30, 50);
                    break;
                case LEFT:
                    robot.moveRobot(.5,40, 50);
                    robot.turnRobot(MotionHardwareG2.Direction.RIGHT, 9, .5, 10);
                    robot.dropperUp();
                    robot.turnRobot(MotionHardwareG2.Direction.LEFT, 18, .5, 10);
                    robot.moveRobot(.5, 25, 50 );
                    break;
                case UNKNOWN:
                    robot.moveRobot(.5,40, 50);
                    robot.turnRobot(MotionHardwareG2.Direction.LEFT, 9, .5, 10);
                    robot.dropperUp();
                    robot.moveRobot(.5, 30, 50);
                    break;
            }
            sleep(20);
            break;
        }
    }
}