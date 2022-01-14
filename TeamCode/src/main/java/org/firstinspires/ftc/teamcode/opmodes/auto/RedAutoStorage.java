package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoStorage extends AutoStorage {
    public RedAutoStorage() {
        super();
        this.multiplier = -1;

        this.bottomRectHeightPercentage = 0.2;
        this.bottomRectWidthPercentage = 0.805;
        this.middleRectHeightPercentage = 0.3;
        this.middleRectWidthPercentage = 0.534;
        this.topRectHeightPercentage = 0.45;
        this.topRectWidthPercentage = 0.195;
    }
}
