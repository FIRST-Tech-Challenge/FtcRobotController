package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RedDrive extends BlueDrive {
    @Override
    public void runOpMode() throws InterruptedException {
        this.power = -1;
        super.runOpMode();
    }
}
