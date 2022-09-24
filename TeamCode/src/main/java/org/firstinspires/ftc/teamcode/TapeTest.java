package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.TapeMeasureBot;

@Autonomous(name="Tape Test", group="Tests")
@Disabled
public class TapeTest extends LinearOpMode {

    protected TapeMeasureBot robot = new TapeMeasureBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        robot.sleep(1000);
        robot.setElevation(0.6);
        robot.sleep(500);
        for (int i = 0; i < 50; i++) {
            robot.controlCoreHex(1, 0);
            robot.onLoop(50, "extending");
        }
        robot.controlCoreHex(0, 0);
        robot.sleep(500);
        robot.setElevation(0.35);
        robot.sleep(500);
//        for (int i = 0; i < 2; i++) {
//            robot.controlSwing(true, false);
//            robot.onLoop(50, "swinging");
//        }
//        robot.controlSwing(false, false);
        robot.sleep(500);
        for (int i = 0; i < 45; i++) {
            robot.controlCoreHex(0, 1);
            robot.onLoop(50, "extending");
        }
        robot.controlCoreHex(0, 0);
        robot.sleep(5000);
    }
}
