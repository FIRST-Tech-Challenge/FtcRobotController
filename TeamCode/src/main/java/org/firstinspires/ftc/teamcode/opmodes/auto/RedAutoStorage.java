package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoStorage extends AutoStorage {
    public RedAutoStorage() {
        super();
        this.multiplier = -1;
        this.isRed = true;
    }
}
