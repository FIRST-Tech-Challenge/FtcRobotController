package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "LeftAutoPark")
public class LeftAutoPark extends BaseAuto {

    @Override
    void parkTerminal() {
        hdw.moveXAxis(-30, 0.5);
    }

    @Override
    void scoreMidPole() {

    }

}