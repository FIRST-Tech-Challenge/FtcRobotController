package org.firstinspires.ftc.teamcode.CompBotV3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EncoderProof extends LinearOpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);

        waitForStart();

        while(!isStopRequested()) {
            r.driveRobotCentric(0, 0, 0);
            telemetry.addData("fl",r.fl.getCurrentPosition());
            telemetry.addData("fr",r.fr.getCurrentPosition());
            telemetry.addData("bl",r.bl.getCurrentPosition());
            telemetry.addData("br",r.br.getCurrentPosition());
            telemetry.update();
        }
    }
}
