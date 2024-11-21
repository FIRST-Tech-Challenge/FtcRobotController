package org.firstinspires.ftc.teamcode.Test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.DriveAcceleration;
import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;
import org.firstinspires.ftc.teamcode.Components.MainDrive;
import org.firstinspires.ftc.teamcode.Components.ViperSlide;
import org.firstinspires.ftc.teamcode.Debug.Debug;

@TeleOp(name = "TeleOpTest", group = "Test")
public class TeleOpTest extends LinearOpMode {
    private double targetPower;
    private double increment;
    private double incrementDividend;
    private boolean isAccelerating;
    private long lastUpdateTime;
    private double currentPower;
    private int updateDelay = 10;

    float frontMultiplier = 1;
    float backMultiplier = 1;

    boolean emergencyStop = false;


    int flipDistanceLimit = 550;
    int slideDistanceLimit = 675;

    Debug debug;

    @Override
    public void runOpMode() {
        // Initialize motors
        HorizontalSlide hSlide = new HorizontalSlide(this, 3);
        ViperSlide viperSlide = new ViperSlide(this);
        DriveAcceleration mainDrive = new DriveAcceleration(this);


        debug = new Debug(this);
        debug.debugMode = true;


        double direction = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !emergencyStop) {
            viperSlide.checkInputs(
                    gamepad2.right_trigger,  // retract
                    gamepad2.left_trigger,   // extend
                    gamepad2.guide,          // reset encoders
                    gamepad2.a,              // hold viper position
                    gamepad2.y,              // bucket rest
                    gamepad2.x               // bucket score
            );
        }

        Log.d("TeleOpTest", "OpMode started");


    }
}