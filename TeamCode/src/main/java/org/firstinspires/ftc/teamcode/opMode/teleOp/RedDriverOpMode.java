package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpRobotController;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

@TeleOp(name = "Red Team - Driver")
public class RedDriverOpMode extends SympleCommandOpMode {
    @Override
    public void initialize() {
        this.robotController = new TeleOpRobotController.Builder()
                .initializeDefaults(this)
                .teamColor(TeamColor.RED)
                .build();
    }
}
