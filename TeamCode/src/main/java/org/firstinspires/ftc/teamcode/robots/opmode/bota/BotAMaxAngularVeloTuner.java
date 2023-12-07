package org.firstinspires.ftc.teamcode.robots.opmode.bota;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.MaxAngularVeloTuner;
import org.firstinspires.ftc.teamcode.robots.BotA2023;

@Autonomous(group = "Robot A", name = "Robot A - MaxAngularVeloTuner")
public class BotAMaxAngularVeloTuner extends MaxAngularVeloTuner {

                @Override
                public void runOpMode() throws InterruptedException {
                    BotA2023 drive = new BotA2023(hardwareMap);
                    super.opModeCode(drive);
                }
}
