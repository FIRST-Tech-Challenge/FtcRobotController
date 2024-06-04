package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.TeamColor;

@TeleOp(name = "Red Team - Driver")
public class RedDriverOpMode extends DriverOpMode {

    protected RedDriverOpMode() {
        super(TeamColor.RED);
    }
}
