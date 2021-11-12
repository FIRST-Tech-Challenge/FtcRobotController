package org.firstinspires.ftc.teamcode.CompBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class motorTest extends LinearOpMode {
    CompBotHW r = new CompBotHW();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();
        r.fl.set(1);
        sleep(1000);
        r.fl.set(0);
        r.fr.set(1);
        sleep(1000);
        r.fr.set(0);
        r.bl.set(1);
        sleep(1000);
        r.bl.set(0);
        r.br.set(1);
        sleep(1000);
        r.br.set(0);

    }
}
