package org.firstinspires.ftc.teamcode.comp.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.GlobalConfig;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_POS;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_COL;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;

@Config
@Autonomous(name = "Auto Test", group = "Test and Calibration")
public class AutoTest extends LinearOpMode {
    public ALLIANCE_POS alliancePos = ALLIANCE_POS.RIGHT;
    public ALLIANCE_COL allianceCol = ALLIANCE_COL.RED;
    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);

    MotionHardware robot = new MotionHardware(this, globalConfig);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        waitForStart();

        while(opModeIsActive()) {
            robot.strafeWithTime(.5, 270, 3);
            robot.strafeWithTime(.5, 45, 2);
            break;
        }
    }
}
