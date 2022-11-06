package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous.TeamColor.RED;
import static org.firstinspires.ftc.teamcode.opmodes.auto.CompetitionAutonomous.TeamSide.LEFT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Left", group = "Autonomous")
public class Red_Left_OpMode extends CompetitionAutonomous{

    public void init() {
        super.init(RED, LEFT);
    }
}