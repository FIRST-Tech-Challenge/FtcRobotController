package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ArmMaintenance extends LinearOpMode {
    RobotMain bart;


    @Override
    public void runOpMode() throws InterruptedException {
        bart = new RobotMain(hardwareMap, telemetry);
        waitForStart();
        bart.output.setComponentPositionsFromOutputEndPoint(new OutputEndPoint(new Point2d(11, 15.3), 0, 0, true));
        while(opModeIsActive()) {
            bart.output.sendVerticalSlidesToTarget();
            telemetry.addLine(bart.output.currentPosition().componentValuesIrl());
            telemetry.update();
        }
    }
}
