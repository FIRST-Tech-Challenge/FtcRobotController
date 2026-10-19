package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Pollen BENCH Park", group = "Pollen")
public final class PollenBenchPark extends PollenAutoBase {
    @Override protected boolean bench() { return true; }
    @Override protected boolean shoot() { return false; }
}
