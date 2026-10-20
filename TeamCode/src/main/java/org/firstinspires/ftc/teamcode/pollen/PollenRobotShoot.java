package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Pollen ROBOT Shoot", group = "Pollen", preselectTeleOp = "PollenBot")
public final class PollenRobotShoot extends PollenAutoBase {
    @Override protected boolean bench() { return false; }
    @Override protected boolean shoot() { return true; }
}
