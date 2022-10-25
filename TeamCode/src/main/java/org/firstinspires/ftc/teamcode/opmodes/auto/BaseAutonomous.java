package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.components.Tensorflow;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;


import java.util.EnumMap;

public abstract class BaseAutonomous extends BaseOpMode {
    Team currentTeam;
    Vuforia vuforia;
    PixyCam pixyCam;

    // Which team are we on?
    public enum Team {
        RED, BLUE
    }

    /**
     * Initialization of systems for autonomous
     * @param team current team in this game
     */
}