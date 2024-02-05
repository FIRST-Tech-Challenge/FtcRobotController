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
                    robot.moveRobot(.5,45, 10);
                    break;
                case RIGHT:
                    robot.moveRobot(.5,35, 10);
                    break;
                case LEFT:
                    robot.moveRobot(.5,25, 10);
                    break;
                case UNKNOWN:
                    robot.moveRobot(.5,15, 10);
                    break;
            }
            sleep(20);
            break;
        }
    }
}