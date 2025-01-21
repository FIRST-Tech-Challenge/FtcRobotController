package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BLue2", group="Auto", preselectTeleOp = "TeleOP")
public class BlueLeft extends LinearOpMode {
    public AutoMethodsTime bot = new AutoMethodsTime();


    //ОСНОВНАЯ ПРОГРАММА
    @Override
    public void runOpMode() throws InterruptedException {
        bot.initC(this);
        waitForStart();
    }
}
