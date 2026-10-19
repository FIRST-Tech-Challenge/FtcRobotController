package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Pollen BENCH Timeout", group = "Pollen")
public final class PollenBenchTimeout extends PollenAutoBase {
    @Override protected boolean bench() { return true; }
    @Override protected boolean shoot() { return true; }
    @Override protected boolean simulateLauncherFailure() { return true; }
}
