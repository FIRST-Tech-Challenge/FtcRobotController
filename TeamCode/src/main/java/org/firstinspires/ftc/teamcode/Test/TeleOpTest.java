package org.firstinspires.ftc.teamcode.Test;

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

    Debug debug;

    @Override
    public void runOpMode() {
        // Initialize motors

        debug = new Debug(this);

//        leftViper = hardwareMap.get(DcMotor.class, "leftViper");
//        rightViper = hardwareMap.get(DcMotor.class, "rightViper");

//        leftViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double direction = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !emergencyStop) {

        }
    }
}