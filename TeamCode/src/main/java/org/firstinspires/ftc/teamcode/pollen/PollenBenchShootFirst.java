package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/** Optional powered-hub check. Simulates speed and maps no hardware. */
@Disabled
@Autonomous(name = "Pollen BENCH Shoot First", group = "Pollen")
public final class PollenBenchShootFirst extends PollenShootFirstAuto {
    @Override protected boolean bench() { return true; }
}
