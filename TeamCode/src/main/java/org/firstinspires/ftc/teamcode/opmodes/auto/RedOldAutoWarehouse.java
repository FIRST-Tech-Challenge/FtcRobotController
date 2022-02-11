package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class RedOldAutoWarehouse extends OldAutoWarehouse {
    public RedOldAutoWarehouse() {
        super();
        this.multiplier = -1;
        this.isRed = true;
    }
}
