package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ConstsForTeleskope;

@Autonomous(name="BlueRight", group="Auto", preselectTeleOp = "TeleOP")
//@Disabled
public class BlueRight extends LinearOpMode implements ConstsForTeleskope {

    public AutoMethodsTime bot = new AutoMethodsTime();

    //ОСНОВНАЯ ПРОГРАММА


    @Override
    public void runOpMode() throws InterruptedException {
        bot.initC(this);
        waitForStart();
    }
}