package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.horizon36596.zenith.ivy.ZenithRobot;

/**
 * Runs the red-authored red-audience route point-mirrored for blue: the file's alliance is RED and this robot runs
 * BLUE, so Ivy applies (x, y, h) to (-x, -y, h + pi) once and aim(frame=AUTHORED) swaps AUDIENCE/FAR.
 * Blue Far start mirrors Red Audience: far wall, facing the blue wall, blue garden ahead. See doc/MatchAutonomousPlan.md and doc/MatchAutonomousPlanCycle.md.
 */
@Autonomous(name = "DRAFT Auto: Blue Far (mirrors red-audience)", group = "Competition", preselectTeleOp = "Mecanum TeleOp - Blue")
public final class BlueFarAutonomous extends BioBuzzAutoFromFile {
    @Override protected String autoName() { return "red-audience"; }
    @Override protected ZenithRobot createRobot() { return new BioBuzzRobot(TurretSubsystem.Alliance.BLUE); }
}
