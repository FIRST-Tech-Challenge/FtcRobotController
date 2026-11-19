package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.horizon36596.zenith.ivy.ZenithRobot;

/**
 * Runs the red-authored red-far-cycle route point-mirrored for blue: the file's alliance is RED and this robot runs
 * BLUE, so Ivy applies (x, y, h) to (-x, -y, h + pi) once and aim(frame=AUTHORED) swaps AUDIENCE/FAR.
 * Blue Audience start mirrors Red Far: audience wall on the left, facing red, flower3 behind. See doc/MatchAutonomousPlan.md and doc/MatchAutonomousPlanCycle.md.
 */
@Autonomous(name = "DRAFT Auto: Blue Audience Cycle (mirrors red-far-cycle)", group = "Competition", preselectTeleOp = "Mecanum TeleOp - Blue")
public final class BlueAudienceCycleAutonomous extends BioBuzzAutoFromFile {
    @Override protected String autoName() { return "red-far-cycle"; }
    @Override protected ZenithRobot createRobot() { return new BioBuzzRobot(TurretSubsystem.Alliance.BLUE); }
}
