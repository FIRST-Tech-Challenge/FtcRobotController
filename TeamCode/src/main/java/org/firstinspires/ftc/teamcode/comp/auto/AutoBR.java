package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.GlobalState.ALLIANCE_POS;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;

@Config
@Autonomous(name = "Auto - BA Right", group = "Auto")
public class AutoBR extends LinearOpMode {

    public ALLIANCE_POS alliancePos = ALLIANCE_POS.RIGHT;
    MotionHardware robot = new MotionHardware(this);
    VisionHardware vision = new VisionHardware(this, alliancePos);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        vision.init();

        robot.moveArm(.5, 5, 5);

        waitForStart();


        // Center Strike Line
        // - Forward: 31.75
        // - Reverse: -44.75
        while(opModeIsActive()) {
            PropPosition propPosition = vision.detectProp();

            //TODO This should be seeing Middle and Right..update trajectories and case
            switch(propPosition) {
                case MIDDLE:
                    //Drop off pixel
                    robot.moveRobot(.5, -38, 10);
                    //Pretend to drop pixel
                    robot.dropPixel();
                    //sleep(1000);
                    //Backup and clear pixel
                    //robot.moveRobot(.5, -5, 5);
                    //Turn to parking location
                    //robot.turnRobot(Direction.LEFT, 16, .5, 10);
                    //Park
                    //robot.moveRobot(.5, 30, 10);
                    break;
                case RIGHT:
                    //Drop off pixel
                    robot.moveRobot(.5, -43.75, 10);
                    //Turn left
                    robot.turnRobot(Direction.LEFT, 6, .5, 10);
                    //Move to line
                    robot.moveRobot(.5, 9, 5);
                    //Drop pixel
                    robot.dropPixel();
                    //Park
                    //robot.moveRobot(.5, 45, 10);
                    //robot.turnRobot(Direction.RIGHT, 14, 5, 10);
                    //robot.moveRobot(.5, 13, 5);
                    break;
                default:
                    //Drop off pixel
                    robot.moveRobot(.5, -30.75, 10);
                    //Turn left
                    robot.turnRobot(Direction.RIGHT, 10, .5, 10);
                    //Move to line
                    robot.moveRobot(.5, 9, 5);
                    //Drop pixel
                    robot.dropPixel();
                    //Move awayn from line
                    //robot.moveRobot(.5, -9, 5);
                    //Turn to park
                    //robot.turnRobot(Direction.RIGHT, 19, 5, 10);
                    //Park
                    //robot.moveRobot(.5, 45, 10);
                    //robot.turnRobot(Direction.RIGHT, 14, 5, 10);
                    //robot.moveRobot(.5, 13, 5);
                    break;

            }
            
            sleep(20);
            break;
        }
    }
}
