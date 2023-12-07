package org.firstinspires.ftc.teamcode.robots.opmode.bota;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.MotorDirectionDebugger;
import org.firstinspires.ftc.teamcode.robots.BotA2023;

@TeleOp(group = "Robot A", name = "Robot A - MotorDirectionDebugger")
public class BotAMotorDirectionDebugger extends MotorDirectionDebugger {

    @Override
    public void runOpMode() throws InterruptedException {
        BotA2023 drive = new BotA2023(hardwareMap);
        super.opModeCode(drive);
    }
}
