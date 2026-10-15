package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Driving - Red Alliance", group = Constants.MAIN_GROUP)
public final class RedAllianceTeleOp extends AllianceDrivingTeleOp {
    public RedAllianceTeleOp(KronBot robot) {
        super(
                robot,
                "RED",
                () -> Constants.RED_BASKET_COMMON_X,
                () -> Constants.RED_BASKET_1_Y,
                () -> Constants.RED_BASKET_COMMON_X,
                () -> Constants.RED_BASKET_2_Y,
                Constants.LIMELIGHT_RED_1_PIPELINE,
                Constants.LIMELIGHT_RED_2_PIPELINE);
    }
}
