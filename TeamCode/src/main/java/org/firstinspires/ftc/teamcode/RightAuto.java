package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "RightAuto")
public class RightAuto extends BaseAuto {

        @Override
        void parkTerminal() {
            hdw.moveXAxis(30, 0.5);
        }

    @Override
    void scoreMidPole () {


    }

}

