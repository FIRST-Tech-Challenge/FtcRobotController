package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="FieldOrientation", group="Linear Opmode")
@Disabled
public class DriveColorExample extends LinearOpMode {

    private IMU imu;

    float redcolor = 0;


    public float getRedcolor() {
        return redcolor;
    }

    boolean colortrue = false;
    private ColorSensor CS;
    private ColorRangeSensor CRS;

    @Override
    public void runOpMode() {

        while (opModeIsActive()) {
            telemetry.addData("Red", "", colortrue, getRedcolor());
            telemetry.update();
        }
    }
}


