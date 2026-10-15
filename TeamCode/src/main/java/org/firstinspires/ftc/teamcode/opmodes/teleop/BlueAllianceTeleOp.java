package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Driving - Blue Alliance", group = Constants.MAIN_GROUP)
public final class BlueAllianceTeleOp extends AllianceDrivingTeleOp {
    public BlueAllianceTeleOp(KronBot robot) {
        super(
                robot,
                "BLUE",
                () -> Constants.BLUE_BASKET_COMMON_X,
                () -> Constants.BLUE_BASKET_1_Y,
                () -> Constants.BLUE_BASKET_COMMON_X,
                () -> Constants.BLUE_BASKET_2_Y,
                Constants.LIMELIGHT_BLUE_1_PIPELINE,
                Constants.LIMELIGHT_BLUE_2_PIPELINE);
    }
}
