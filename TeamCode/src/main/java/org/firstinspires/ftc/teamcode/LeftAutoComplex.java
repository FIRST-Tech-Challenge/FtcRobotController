package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "LeftAutoComplex")
public class LeftAutoComplex extends BaseAuto {

    @Override
    void parkTerminal() {
        hdw.moveYAxis(-6, -0.3);
        hdw.moveXAxis(-26, 0.5);
    }

    @Override
    void scoreMidPole () {
        YawPitchRollAngles orientation = hdw.imu.getRobotYawPitchRollAngles();
        double initHeading = orientation.getYaw(AngleUnit.DEGREES);

        hdw.moveYAxis(24, 0.3);
        hdw.moveYAxis(-5, -0.3);
        //Get initial heading

        Log.d("9010", "before turn heading: " + initHeading);

        hdw.turn (-45);
        hdw.moveXAxis(2,0.5);
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        hdw.moveYAxis(10, 0.3);
        hdw.dropCone();
        hdw.goToHeight(Hardware2022.SlideHeight.Mid);
        hdw.moveYAxis(-10, -0.2);

        //Turn back to init heading.
        orientation = hdw.imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw(AngleUnit.DEGREES);
        Log.d("9010", "after Op heading: " + currentHeading);
        Log.d("9010", "Turn back for : " + (initHeading - currentHeading) );

        hdw.turn(initHeading - currentHeading);

        orientation = hdw.imu.getRobotYawPitchRollAngles();
        currentHeading = orientation.getYaw(AngleUnit.DEGREES);
        Log.d("9010", "before turn heading: " + initHeading);
        Log.d("9010", "after trun back heading : " + currentHeading);

    }

    @Override
    void parkZone1( ) {
        telemetry.addData("Park zone 1 ", this.currentSide);
        telemetry.update();
        //Move Left
        //hdw.moveYAxis(-2, -0.3);
        hdw.moveXAxis( -22.0, 0.3);
        hdw.goToHeight(Hardware2022.SlideHeight.Ground);
    }

    @Override
    void parkZone2( ) {
        telemetry.addData("Park zone 2 ", this.currentSide);
        telemetry.update();
        //Stay in place
        hdw.goToHeight(Hardware2022.SlideHeight.Ground);

    }

    @Override
    void parkZone3( ) {
        telemetry.addData("Park zone 3 ", this.currentSide);
        telemetry.update();
        //Move right
        //hdw.moveYAxis(-2.0, -0.3);
        hdw.moveXAxis( 22.0, 0.3);
        hdw.goToHeight(Hardware2022.SlideHeight.Ground);

    }

}