package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveRobotUsingController();
            controlArmPower();
            robot.updatePosition();
            /*telemetry.addData("Current X", robot.currentX);
            telemetry.addData("Current Y", robot.currentY);
            telemetry.addData("Current heading", robot.curAngle);
            telemetry.update();*/
            idle();
        }


    }
}
