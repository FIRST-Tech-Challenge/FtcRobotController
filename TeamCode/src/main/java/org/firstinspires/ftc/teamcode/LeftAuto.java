package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LeftAuto")
@Disabled
public class LeftAuto extends BaseAuto {

    /**
     * THis is the movement from the left side of field to park
     */

    void movePark() {
        hdw.moveYAxis(6.0,0.7);

    }

}