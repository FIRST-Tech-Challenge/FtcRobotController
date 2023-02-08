package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAutoPark")
public class TestAutoPark extends BaseAuto {

    @Override
    void parkTerminal() {

    }

    @Override
    void scoreMidPole() {

    }

    @Override
    void parkZone1() {
        telemetry.addData("Park zone 1 ", this.currentSide);
        telemetry.update();
        Log.d("9010", "Park Zone 1");

    }

    @Override
    void parkZone2() {
        telemetry.addData("Park zone 2 ", this.currentSide);
        telemetry.update();
        Log.d("9010", "Park Zone 2");

    }

    @Override
    void parkZone3() {
        telemetry.addData("Park zone 3 ", this.currentSide);
        telemetry.update();
        Log.d("9010", "Park Zone 3");

    }

}
