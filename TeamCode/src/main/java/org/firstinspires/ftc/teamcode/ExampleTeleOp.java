package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class ExampleTeleOp extends LinearOpMode{
    private Robot robot = new Robot();
    @Override
    public void runOpMode() {
        robot.init(this);
        waitForStart();
        while (opModeIsActive()) {
            robot.handleRobotMove();
        }
    }
}
