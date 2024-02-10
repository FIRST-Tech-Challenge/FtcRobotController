package org.firstinspires.ftc.teamcode.comp.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.GlobalConfig;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_POS;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_COL;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "Auto - RA Right", group = "Auto")
public class AutoRR extends LinearOpMode {

    public ALLIANCE_POS alliancePos = ALLIANCE_POS.RIGHT;
    public ALLIANCE_COL allianceCol = ALLIANCE_COL.RED;
    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);

    private static int DESIRED_TAG_ID = 0;

    private static int DESIRED_DISTANCE = 6;

    private AprilTagDetection desiredTag = null;
    MotionHardware robot = new MotionHardware(this, globalConfig);
    VisionHardware vision = new VisionHardware(this, alliancePos);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        vision.init();

        robot.moveArm(.5, 5, 5);

        waitForStart();

        //TODO In parking code, drop arms to TeleOp pickup position

        // Center Strike Line
        // - Forward: 31.75
        // - Reverse: -44.75
        while(opModeIsActive()) {
            vision.resetExposure();
            PropPosition propPosition = vision.detectProp();
            // Left = 4, Middle = 5, Right = 6

            switch(propPosition) {
                case MIDDLE:
                    DESIRED_TAG_ID = 5;
                    //Drop off pixel
                    //robot.moveRobot(.5, -3, 10);
                    //robot.moveArm(.5, -10, 5);
                    //robot.moveRobot(.5, -36, 10);
                    //robot.moveRobot(.5, -38, 10);

                    // Dropper Mode
                    robot.moveRobot(.5, -15, 10);
                    robot.dropPixel();
                    sleep(1000);
                    robot.moveRobot(.5, 2, 10);

                    // Turn to face camera to board so we can find april tag
                    robot.turnRobot(Direction.LEFT, 13, .5, 10);
                    robot.moveRobot(.5, 21, 10);
                    robot.moveArmMotorToPosition(-1200, 10);
                    robot.moveRobot(.5, -3, 10);






                    //vision.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur


                    //vision.detectTag(DESIRED_TAG_ID, robot, 10);
                    /**
                    if(desiredTag == null) {
                        // go park because we didn't find the tag
                        telemetry.addData("Tag Not Found ","%7d", DESIRED_TAG_ID);
                        telemetry.update();
                    } else {
                        robot.navToTag(desiredTag, DESIRED_DISTANCE);
                    }
                     **/


                    //Backup and clear pixel
                    //robot.moveRobot(.5, -3.75, 5);
                    //robot.moveRobot(.5, -5, 5);

                    //Turn to parking location
                    //robot.turnRobot(Direction.LEFT, 9, .5, 10);
                    //Park
                    //robot.moveRobot(.5, 28, 10);
                    break;
                case RIGHT:
                    //Dropper Mode
                    robot.moveRobot(.5, -13, 10);
                    robot.turnRobot(Direction.RIGHT, 7, .5, 10);
                    robot.dropPixel();
                    robot.moveRobot(.5, 3, 10);
                    robot.turnRobot(Direction.LEFT, 13, .5, 10);
                    robot.moveRobot(.5, 2, 10);
                    robot.turnRobot(Direction.LEFT, 6.5, .5, 10);
                    robot.moveRobot(.5, 22, 10);
                    robot.moveArmMotorToPosition(-1200, 10);
                    robot.moveRobot(.5, -3, 10);

                    //Drop off pixel
                    //robot.moveRobot(.5, -43.75, 10);
                    //Turn left
                    //robot.turnRobot(Direction.LEFT, 6, .5, 10);
                    //Move to line
                    //robot.moveRobot(.5, 9, 5);
                    //Drop pixel
                    //robot.dropPixel();
                    //Park
                    //robot.moveRobot(.5, 45, 10);
                    //robot.turnRobot(Direction.LEFT, 14, 5, 10);
                    //robot.moveRobot(.5, 13, 5);
                    break;
                default:
                    //Dropper Mode
                    robot.moveRobot(.5, -13, 10);
                    robot.turnRobot(Direction.LEFT, 9, .5, 10);
                    robot.moveRobot(.5, -3, 10);
                    robot.dropPixel();
                    robot.moveRobot(.5, 2, 10);
                    robot.turnRobot(Direction.LEFT, 6, .5, 10);
                    robot.moveRobot(.5, 9, 10);
                    robot.turnRobot(Direction.RIGHT, 3, .5, 10);
                    robot.moveRobot(.5, 15, 10);
                    robot.moveArmMotorToPosition(-1200, 10);
                    robot.moveRobot(.5, -3, 10);

                    //Drop off pixel
                    //robot.moveRobot(.5, -30.75, 10);
                    //Turn left
                    //robot.turnRobot(Direction.RIGHT, 14, .5, 10);
                    //Move to line
                    //robot.moveRobot(.5, 9, 5);
                    //Drop pixel
                    //robot.dropPixel();
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
