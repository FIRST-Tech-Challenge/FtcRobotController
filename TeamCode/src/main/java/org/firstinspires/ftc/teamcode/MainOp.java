package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** @noinspection unused */
@TeleOp(name = "MainOp", group = "TeleOp")
public class MainOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        BaseRobot baseRobot = new BaseRobot(hardwareMap, gamepad1, gamepad2, this, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            baseRobot.driveGamepads();

            if (Settings.Deploy.ODOMETRY) {
                baseRobot.odometry.update();
            }
        }
        baseRobot.shutDown();
    }

}
