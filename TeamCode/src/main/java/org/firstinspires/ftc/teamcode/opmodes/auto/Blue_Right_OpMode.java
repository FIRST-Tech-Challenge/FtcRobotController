package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous.TeamColor.BLUE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous.TeamSide.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Blue Right", group = "Autonomous")
public class Blue_Right_OpMode extends CompetitionAutonomous{

    public void init() {
        super.init(BLUE, RIGHT);
    }
}