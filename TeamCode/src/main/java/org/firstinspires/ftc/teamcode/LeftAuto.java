package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "LeftAuto")
public class LeftAuto extends BaseAuto {

    @Override
    void parkTerminal() {
        hdw.moveXAxis(-30, 0.5);
    }

    @Override
    void scoreMidPole () {
        YawPitchRollAngles orientation = hdw.imu.getRobotYawPitchRollAngles();
        double initHeading = orientation.getYaw(AngleUnit.DEGREES);

        hdw.moveYAxis(33, 0.3);
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        //Get initial heading

        Log.d("9010", "before turn heading: " + initHeading);

        hdw.turn (-45);
        hdw.moveYAxis(10, 0.3);
        hdw.dropCone();
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        hdw.moveYAxis(-7, -0.3);
        hdw.goToHeight(Hardware2022.SlideHeight.Ground);

        //Turn back to init heading.
        orientation = hdw.imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw(AngleUnit.DEGREES);
        Log.d("9010", "after turn heading: " + currentHeading);

        hdw.turn(initHeading - currentHeading);

    }
}