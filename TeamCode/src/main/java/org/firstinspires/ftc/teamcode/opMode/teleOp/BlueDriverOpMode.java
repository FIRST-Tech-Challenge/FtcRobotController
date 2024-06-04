package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.TeamColor;

@TeleOp(name = "Blue Team - Driver")
public class BlueDriverOpMode extends DriverOpMode {

    protected BlueDriverOpMode() {
        super(TeamColor.BLUE);
    }
}
