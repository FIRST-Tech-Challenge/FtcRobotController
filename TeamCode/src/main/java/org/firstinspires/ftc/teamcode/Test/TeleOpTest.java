package org.firstinspires.ftc.teamcode.Test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;
import org.firstinspires.ftc.teamcode.Debug.Debug;

@TeleOp(name = "TeleOpTest", group = "Test")
public class TeleOpTest extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    private DcMotor leftViper;
    private DcMotor rightViper;

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


        debug = new Debug(this);

        double direction = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !emergencyStop) {

            if(gamepad1.guide) {
                telemetry.addData("giude button: ", "pressed");
                telemetry.update();
                Log.d("TeleOpTest", "guide button pressed");
            }

        }

        Log.d("TeleOpTest", "OpMode started");


    }
}