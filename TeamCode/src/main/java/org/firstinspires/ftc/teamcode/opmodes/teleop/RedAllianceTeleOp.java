package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name = "Driving - Red Alliance", group = Constants.MAIN_GROUP)
public final class RedAllianceTeleOp extends AllianceDrivingTeleOp {
    public RedAllianceTeleOp() {
        super(
                "RED",
                () -> Constants.RED_BASKET_COMMON_X,
                () -> Constants.RED_BASKET_1_Y,
                () -> Constants.RED_BASKET_COMMON_X,
                () -> Constants.RED_BASKET_2_Y);
    }
}
