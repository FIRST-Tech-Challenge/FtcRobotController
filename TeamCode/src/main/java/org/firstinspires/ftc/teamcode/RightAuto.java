package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "RightAuto")
public class RightAuto extends BaseAuto {

        @Override
        void parkTerminal() {
            hdw.moveXAxis(30, 0.5);
        }

    @Override
    void scoreMidPole () {
        hdw.moveYAxis(33, 0.3);
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        hdw.turn (45);
        hdw.moveYAxis(10, 0.3);
        hdw.dropCone();
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        hdw.moveYAxis(-7, -0.3);
        hdw.goToHeight(Hardware2022.SlideHeight.Ground);
        hdw.turn(-45);
    }

}

