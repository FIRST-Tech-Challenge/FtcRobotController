package org.firstinspires.ftc.teamcode.opModes.team1.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;

@Autonomous(name = "Red auto", group = "Autos")
public class BlueAuto extends AutonomousLinearModeBase {
    @Override
    public void run() {
        GenericAuto auto = new GenericAuto();
        auto.run(TeamColour.BLUE, this);
    }
}
