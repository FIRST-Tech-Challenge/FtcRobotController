package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="BIOBUZZ Shared Java Blue",group="Quantum Storm")
public final class BlueSharedAuto extends SharedBioBuzzAuto {
    @Override protected int alliance() { return 1; }
}
