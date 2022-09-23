package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.TeamState;

@TeleOp(name = "DriveOpMode - RED", group = "TeleOp")
public class DriveOpModeRed extends DriveOpMode {
    @Override
    public void init() {
        super.init(TeamState.RED);
    }
}
