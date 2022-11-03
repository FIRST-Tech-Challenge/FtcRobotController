package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LeftAuto")
public class LeftAuto extends BaseAuto {

    @Override
    public void runOpMode() {
        super.runOpMode();
    }

    /**
     * THis is the movement from the left side of field to park
     */
    @Override
    void movePark() {
        hdw.moveXAxis(6.0,0.7);

    }


}