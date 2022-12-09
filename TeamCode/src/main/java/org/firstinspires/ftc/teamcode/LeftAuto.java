package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "LeftAuto")
public class LeftAuto extends BaseAuto {

    @Override
    void parkTerminal() {
        hdw.moveXAxis(-30, 0.5);
    }
}