package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Pollen BENCH Shoot", group = "Pollen")
public final class PollenBenchShoot extends PollenAutoBase {
    @Override protected boolean bench() { return true; }
    @Override protected boolean shoot() { return true; }
}
