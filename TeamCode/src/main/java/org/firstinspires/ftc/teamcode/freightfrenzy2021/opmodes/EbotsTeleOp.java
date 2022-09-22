package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsColorSensor;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Arm;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Carousel;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.EbotsMotionController;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.MecanumDrive;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

@Disabled
@TeleOp
public class EbotsTeleOp extends LinearOpMode {

    private EbotsMotionController motionController;
    private StopWatch lockoutStopWatch = new StopWatch();
    private StopWatch endGameStopWatch = new StopWatch();
    private Telemetry.Item zeroHeadingItem = null;
    private Intake intake;
    private EbotsColorSensor colorSensor;
    private Carousel carousel;
    private Arm arm;
    public Bucket bucket;

    private DistanceSensor distanceSensor;
    private boolean endGameRumbleIssued;
    private boolean justDumped = false;


    public void setJustDumped(boolean justDumped) {
        this.justDumped = justDumped;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        endGameRumbleIssued = false;
        intake = Intake.getInstance(hardwareMap);
        carousel = Carousel.getInstance(hardwareMap);
        bucket = Bucket.getInstance(this);
        arm = Arm.getInstance(this);
        //Initialize Arm with new HardwareMap
        //arm.init(hardwareMap); moved to UtilFuncs
        UtilFuncs.initManips(arm,carousel,this);
        colorSensor = new EbotsColorSensor(hardwareMap);

        motionController = EbotsMotionController.get(MecanumDrive.class, this);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");

        while (! this.isStarted()){
            this.handleUserInput(gamepad1);
            updateTelemetry();
        }

        waitForStart();
        endGameStopWatch.reset();

        while (opModeIsActive()){

            rumbleIfEndGame();

            this.handleUserInput(gamepad1);
            motionController.handleUserInput(gamepad1);
            intake.handleUserInput(gamepad2);
            carousel.handleUserInput(gamepad2);
            bucket.handleUserInput(gamepad2);
            if(this.justDumped) {
                arm.moveToLevel(Arm.Level.ONE);
                justDumped = false;
            }
            arm.handleUserInput(gamepad2);

            updateTelemetry();
        }
    }

    private void rumbleIfEndGame() {
        if (endGameStopWatch.getElapsedTimeSeconds() >= 89 && !endGameRumbleIssued){
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
            endGameRumbleIssued = true;
        }
    }

    private void updateTelemetry() {
        String twoDecimals = "%.2f";
        Telemetry.Item zeroHeadingLine = null;
        telemetry.addData("Motion Controller", motionController.getName());
        telemetry.addData("Current Distance", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Carousel Speed (fmt)", String.format(twoDecimals, carousel.getPower()));
        telemetry.addData("Intake Speed", String.format(twoDecimals, intake.getSpeed()));
        telemetry.addData("Arm isAtBottom", arm.isAtBottom());
        telemetry.addData("Arm position", arm.getPosition());
        telemetry.addData("Arm is zeroed ", arm.getIsZeroed());
        telemetry.addData("Hue ", String.format(twoDecimals, colorSensor.getHue()));

        if (motionController instanceof FieldOrientedDrive){
            telemetry.addData("Field Heading", String.format(twoDecimals, ((FieldOrientedDrive) motionController).getCurrentHeadingDeg()));
            telemetry.addData("Initial Heading", String.format(twoDecimals, ((FieldOrientedDrive) motionController).getZeroHeadingDeg()));
        }
        telemetry.update();
    }

    private void handleUserInput(Gamepad gamepad){
        boolean lockoutActive = lockoutStopWatch.getElapsedTimeMillis() < 600;

        if (lockoutActive){
            return;
        }


        if(gamepad.left_bumper && gamepad.right_stick_button){
            if (motionController instanceof MecanumDrive){
                motionController = EbotsMotionController.get(FieldOrientedDrive.class, this);
            } else if (motionController instanceof FieldOrientedDrive){
                motionController = EbotsMotionController.get(MecanumDrive.class, this);
            }

            gamepad.rumble(1.0, 1.0, 400);  // 200 mSec burst on left motor.
            lockoutStopWatch.reset();
        }

    }
}

