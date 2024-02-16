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
                    // Dropper Mode
                    robot.moveRobot(.5, -15, 10);
                    robot.dropPixel();
                    sleep(1000);
                    robot.moveRobot(.5, 2, 10);
                    robot.turnRobot(Direction.LEFT, 13, .5, 10);
                    robot.moveRobot(.5, 21, 10);
                    robot.moveArmMotorToPosition(-1200, 10);
                    robot.moveRobot(.5, -3, 10);
                    robot.strafeWithTime(.5, 180, 2);
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
                    robot.strafeWithTime(.5, 180, 2);
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
                    robot.strafeWithTime(.5, 180, 2);
                    break;
            }

            sleep(20);
            break;
        }
    }
}
