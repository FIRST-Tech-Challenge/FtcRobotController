package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="BIOBUZZ Shared Java Red",group="Quantum Storm")
public final class RedSharedAuto extends SharedBioBuzzAuto {
    @Override protected int alliance() { return 0; }
}
