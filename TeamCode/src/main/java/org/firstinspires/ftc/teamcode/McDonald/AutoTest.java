package org.firstinspires.ftc.teamcode.McDonald;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;

@Config
@Autonomous(name = "Auto - RA Right", group = "Auto")
public class AutoTest extends LinearOpMode {

    MotionHardware robot = new MotionHardware(this);
    VisionHardware vision = new VisionHardware(this);
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
            PropPosition propPosition = PropPosition.RIGHT;

            switch(propPosition) {
                case MIDDLE:
                    //Drop off pixel
                    robot.moveRobot(.5, -43.75, 10);
                    //Pretend to drop pixel
                    robot.dropPixel();
                    sleep(1000);
                    //Backup and clear pixel
                    robot.moveRobot(.5, -5, 5);
                    //Turn to parking location
                    robot.turnRobot(Direction.LEFT, 16, .5, 10);
                    //Park
                    robot.moveRobot(.5, 30, 10);
                    break;
                case RIGHT:
                    //Drop off pixel
                    robot.moveRobot(.5, -43.75, 10);
                    //Turn left
                    robot.turnRobot(Direction.LEFT, 10, .5, 10);
                    //Move to line
                    robot.moveRobot(.5, 8, 5);
                    //Drop pixel
                    robot.dropPixel();
                    //Park
                    robot.moveRobot(.5, 40, 10);
                    robot.turnRobot(Direction.LEFT, 14, 5, 10);
                    robot.moveRobot(.5, 15, 5);
                    break;
                default:
                    //Drop off pixel
                    robot.moveRobot(.5, -43.75, 10);
                    //Pretend to drop pixel
                    robot.dropPixel();
                    sleep(1000);
                    //Backup and clear pixel
                    robot.moveRobot(.5, -5, 5);
                    //Turn to parking location
                    robot.turnRobot(Direction.LEFT, 16, .5, 10);
                    //Park
                    robot.moveRobot(.5, 30, 10);
                    break;
            }



            sleep(20);
            break;
        }
    }
}
