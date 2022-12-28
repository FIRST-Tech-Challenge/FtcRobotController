package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "LeftAuto")
public class LeftAuto extends BaseAuto {

    @Override
    void parkTerminal() {
        hdw.moveXAxis(-30, 0.5);
    }

    @Override
    void scoreMidPole () {
        hdw.moveYAxis(30, 0.3);
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        hdw.turn (-45);
        hdw.moveYAxis(1, 0.1);
        hdw.dropCone();
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        hdw.moveYAxis(-1, -0.1);
        hdw.turn(45);
        hdw.moveYAxis(-31, -0.3);


    }
}