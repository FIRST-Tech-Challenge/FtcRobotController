package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.GlobalConfig;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_POS;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;

@Config
@Autonomous(name = "Auto - RA Left", group = "Auto")
public class AutoRL extends LinearOpMode {

    public ALLIANCE_POS alliancePos = ALLIANCE_POS.LEFT;
    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);
    MotionHardware robot = new MotionHardware(this, globalConfig);
    VisionHardware vision = new VisionHardware(this, alliancePos);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        vision.init();

        robot.moveArm(.5, 5, 5);

        waitForStart();

        //TODO Add Park from RL

        // Center Strike Line
        // - Forward: 31.75
        // - Reverse: -44.75
        while(opModeIsActive()) {
            PropPosition propPosition = vision.detectProp();

            switch(propPosition) {
                case MIDDLE:
                    //Drop off pixel
                    //robot.moveRobot(.5, -38, 10);
                    //Pretend to drop pixel
                    //robot.dropPixel();
                    //sleep(1000);
                    //Backup and clear pixel
                    //robot.moveRobot(.5, -4.75, 5);
                    //Turn to parking location
                    //robot.turnRobot(Direction.LEFT, 20, .5, 10);
                    //Park
                    //robot.moveRobot(.5, 95, 10);
                    robot.moveRobot(.5, -15, 10);
                    robot.dropPixel();
                    sleep(1000);
                    //robot.moveRobot(.5, 14.3, 10);
                    //robot.turnRobot(Direction.RIGHT, 16, .5, 10);
                    //robot.moveRobot(.5, 75, 10);
                    //robot.moveArm(.5, 2, 10);
                    break;
                case LEFT:
                    //Dropper Mode
                    robot.moveRobot(.5, -13, 10);
                    robot.turnRobot(Direction.LEFT, 7, .5, 10);
                    robot.dropPixel();
                    //robot.moveRobot(.5, -9, 10);
                    //robot.turnRobot(Direction.LEFT, 6, .5, 10);
                    //robot.moveRobot(.5, 36.75, 10);
                    //robot.turnRobot(Direction.RIGHT, 16, .5, 10);
                    //robot.moveRobot(.5, 75, 10);
                    //robot.moveArm(.5, 2, 10);
                    //Park
                    //robot.moveRobot(.5, 45, 10);
                    //robot.turnRobot(Direction.LEFT, 14, 5, 10);
                    //robot.moveRobot(.5, 13, 5);
                    break;
                default:
                    //Dropper Mode
                    robot.moveRobot(.5, -13, 10);
                    robot.turnRobot(Direction.RIGHT, 9, .5, 10);
                    robot.moveRobot(.5, -3, 10);
                    robot.dropPixel();
                    //robot.moveRobot(.5, -8, 5);
                    //robot.turnRobot(Direction.RIGHT, 14, .5, 10);
                    //robot.moveRobot(.5, 28.25, 10);
                    //robot.turnRobot(Direction.LEFT, 30, .5, 10);
                    //robot.moveRobot(.5, 75, 10);
                    //Move awayn from line
                    //robot.moveRobot(.5, -9, 5);
                    //Turn to park
                    //robot.turnRobot(Direction.LEFT, 19, 5, 10);
                    //Park
                    //robot.moveRobot(.5, 45, 10);
                    //robot.turnRobot(Direction.LEFT, 14, 5, 10);
                    //robot.moveRobot(.5, 13, 5);
                    break;
            }

            sleep(20);
            break;
        }
    }
}
