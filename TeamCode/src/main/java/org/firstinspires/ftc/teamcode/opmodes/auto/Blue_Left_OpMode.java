package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous.TeamColor.BLUE;
import static org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous.TeamSide.LEFT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Left", group = "Autonomous")
public class Blue_Left_OpMode extends CompetitionAutonomous{

    public void init() {
        super.init(BLUE, LEFT);
    }
}