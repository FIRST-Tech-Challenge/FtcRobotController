package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous.TeamColor.RED;
import static org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous.TeamSide.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Right", group = "Autonomous")

public class Red_Right_OpMode extends CompetitionAutonomous{

    public void init() {
        super.init(RED, RIGHT);
    }
}