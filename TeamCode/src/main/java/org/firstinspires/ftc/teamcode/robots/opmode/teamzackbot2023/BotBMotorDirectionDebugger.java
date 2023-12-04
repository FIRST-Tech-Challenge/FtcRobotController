package org.firstinspires.ftc.teamcode.robots.opmode.teamzackbot2023;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.MotorDirectionDebugger;
import org.firstinspires.ftc.teamcode.robots.BotB2023;

@TeleOp(group = "teamzackbot2023", name = "Robot B - MotorDirectionDebugger")
public class BotBMotorDirectionDebugger extends MotorDirectionDebugger {

    @Override
    public void runOpMode() throws InterruptedException {
        BotB2023 drive = new BotB2023(hardwareMap);
        super.opModeCode(drive);
    }
}
