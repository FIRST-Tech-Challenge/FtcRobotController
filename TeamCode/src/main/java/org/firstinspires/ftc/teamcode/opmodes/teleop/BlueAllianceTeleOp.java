package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp(name = "Driving - Blue Alliance", group = Constants.MAIN_GROUP)
public final class BlueAllianceTeleOp extends AllianceDrivingTeleOp {
    public BlueAllianceTeleOp() {
        super(
                "BLUE",
                () -> Constants.BLUE_BASKET_COMMON_X,
                () -> Constants.BLUE_BASKET_1_Y,
                () -> Constants.BLUE_BASKET_COMMON_X,
                () -> Constants.BLUE_BASKET_2_Y);
    }
}
