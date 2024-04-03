package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "BucketBotV1", group = "LinearOpMode")
public class BucketBotV1 extends LinearOpMode {

    @Override
    public void runOpMode() {
        BucketBot bucketBot = new BucketBot(hardwareMap);

        boolean toggle = false;
        ElapsedTime toggleTime = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {
            bucketBot.drivetrain.drive(gamepad1.left_stick_y/2.0, gamepad1.right_stick_y/2.0);

            if (gamepad1.x && toggleTime.time() > .75 && !toggle) {
                toggle = true;
                toggleTime.reset(); }
            else if (gamepad1.x && toggleTime.time() > .75 && toggle) {
                toggle = false;
                toggleTime.reset();
            }

            bucketBot.bucketbotLid.openLid(toggle);
        }
    }
}
