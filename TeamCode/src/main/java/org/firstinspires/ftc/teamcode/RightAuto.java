package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "RightAuto")
public class RightAuto extends BaseAuto {

        @Override
        void parkTerminal() {
            hdw.moveXAxis(30, 0.5);
        }

    @Override
    void scoreMidPole () {
        YawPitchRollAngles orientation = hdw.imu.getRobotYawPitchRollAngles();
        double initHeading = orientation.getYaw(AngleUnit.DEGREES);

        hdw.moveYAxis(48, 0.3);
        hdw.moveYAxis(-6, -0.3);
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        //Get initial heading

        Log.d("9010", "before turn heading: " + initHeading);

        hdw.turn (45);
        hdw.moveYAxis(12, 0.3);
        hdw.dropCone();
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        hdw.moveYAxis(-7, -0.3);

        //Turn back to init heading.
        orientation = hdw.imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw(AngleUnit.DEGREES);
        Log.d("9010", "after Op heading: " + currentHeading);
        Log.d("9010", "Turn back for : " + (initHeading - currentHeading) );

        hdw.turn(initHeading - currentHeading);

        orientation = hdw.imu.getRobotYawPitchRollAngles();
        currentHeading = orientation.getYaw(AngleUnit.DEGREES);
        Log.d("9010", "after trun back heading (Init) : " + currentHeading);


    }

}

