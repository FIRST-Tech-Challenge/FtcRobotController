package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NueralNetworks.Classes.NNFileReader;
import org.firstinspires.ftc.teamcode.Utils.Detection.CheckedDistanceSensor;
import org.firstinspires.ftc.teamcode.Utils.Detection.DistSensorObstacleDetector;
import org.firstinspires.ftc.teamcode.Utils.Detection.LimelightBallDetection;
import org.firstinspires.ftc.teamcode.Utils.Movement.PPRPHybrid;
import org.firstinspires.ftc.teamcode.Utils.Movement.Pathfinder;
import org.firstinspires.ftc.teamcode.Utils.Movement.PosGetters.PinpointPosGet;
import org.firstinspires.ftc.teamcode.Utils.Movement.PosGetters.PositionGetter;
import org.firstinspires.ftc.teamcode.Utils.Threads.SensingThread;

import java.io.File;
import java.util.Random;

public abstract class LinearOpMode2026 extends LinearOpMode {

    public final double TICKS_PER_REV = 28;
    public final double INTAKE_POWER = 0.7; // was 0.8
    public final double LIFT_POWER = 0.7; //Was better at 0.7 but trying 0.6 // Was 1 but missed when shooting 4
    public Servo lla;
    public PositionGetter posGet;
    /**BL BR FL FR*/
    public DcMotor[] mot;
    public String configPath = MiscUtils.dataFolder+"/config2026to2027.robocfg";
    public File loggerFile = new File(MiscUtils.dataFolder+"/log.robolog");
    public Logger log = new Logger(loggerFile);
    public Limelight3A limelight;
    public DistanceSensor dsl, dsr;
    public CheckedDistanceSensor dslC, dsrC;
    public SensingThread sensing = new SensingThread(this);
    public DcMotorEx turret;
    public static boolean isCurrentAllienceRed = true;
    public static boolean isCompetition = true;
    public LimelightBallDetection ballDetection;

    // public IMU imu;

    public DcMotorEx flywheel, lift, intake;

    static final double FLYWHEEL_P = 173.2;
    static final double FLYWHEEL_F = 17.19;

    public Servo hood;

    public Servo il;

    /**
     * Call this to configure motors and the position getter...
     */
    public void config() {
        String[] dailyMessage = {"We're no strangers to looove, you know the rules and so do i. A full commitment is what I'm thinking of, you won't get this far from any other guy, I, just want to tell you how I'm feeling. Just wanna make you understand, never gunna give you up, never gunna let you down, never gunna run around and desert you, never gunna make you cry, never gunna say goodbye! never gunna tell a lie and hurt you!",
                                "Hello?",
                                "I've been alone with you inside my mind, And in my dreams, I've kissed your lips a thousand times, I sometimes see you pass outside my door, Hello, is it me you're looking for? I can see it in your eyes, I can see it in your smile, You're all I've ever wanted, And my arms are open wide, 'Cause you know just what to say, And you know just what to do, And I want to tell you so much, I love you... I long to see the sunlight in your hair, And tell you time and time again how much I care, Sometimes, I feel my heart will overflow... Hello, I've just got to let you know, 'Cause I wonder where you are, And I wonder what you do, Are you somewhere feeling lonely? Or is someone loving you? Tell me how to win your heart, For I haven't got a clue, But let me start by saying, I love you... Hello, is it me you're looking for? 'Cause I wonder where you are, And I wonder what you do, Are you somewhere feeling lonely? Or is someone loving you, Tell me how to win your heart, For I haven't got a clue, But let me start by saying, I love you...",
                                "Where do you live?",
                                "THIS IS NO EASTER EGG, THIS IS A BUG!",
                                "Are you a true Hammerhead?",
        };

        NNFileReader.telemetry = telemetry;
        Pathfinder.telemetry = telemetry;
        DistSensorObstacleDetector.telemetry = telemetry;
        PPRPHybrid.tem = telemetry;

        Random random = new Random();
        boolean doMessage = random.nextFloat() > 0.9;

        try {
            isCurrentAllienceRed = MiscUtils.readConfig(configPath, (byte) 2) == 0;
            isCompetition = MiscUtils.readConfig(configPath, (byte) 3) == 0;
        } catch (Exception e) {}

        telemetry.addLine("Current Alliance: " + (isCurrentAllienceRed ? "Red" : "Blue"));

        /*while (!gamepad1.a){
            telemetry.addLine("Current Alliance: " + (isCurrentAllienceRed ? "Red" : "Blue"));
            if (gamepad1.b){
                isCurrentAllienceRed = !isCurrentAllienceRed;
                while(gamepad1.b){

                }
            }

            telemetry.update();
        }*/




        if (doMessage && MiscUtils.readConfig(configPath, (byte) 0) == 0){
            telemetry.speak(dailyMessage[random.nextInt(dailyMessage.length)]);
        }

        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");

            // limelight.updateRobotOrientation(imu.getYaw(Degrees));

            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.start(); // DO THIS RIGHT HERE? Possibly Do this at start
            ballDetection = new LimelightBallDetection(limelight);
        } catch (Exception e) {
            telemetry.addData("Config error","Limelight not found: " + e);
        }

        try {
            lla = hardwareMap.get(Servo.class, "LLA");
            il = hardwareMap.get(Servo.class, "IL");
        }catch (Exception e){
            telemetry.addData("Config error", "LLA not found: " + e);
        }

        try {
            mot = new DcMotor[]{
                    hardwareMap.get(DcMotorEx.class, "BL"), //back left
                    hardwareMap.get(DcMotorEx.class, "BR"), //back right
                    hardwareMap.get(DcMotorEx.class, "FL"), //front left
                    hardwareMap.get(DcMotorEx.class, "FR") //front right
            };
            mot[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mot[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mot[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mot[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mot[0].setDirection(DcMotorSimple.Direction.REVERSE);
            mot[1].setDirection(DcMotorSimple.Direction.REVERSE);
            mot[2].setDirection(DcMotorSimple.Direction.REVERSE);
            mot[3].setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Config error","Motors not found: " + e);
        }
        try {
            GoBildaPinpointDriver pin = hardwareMap.get(GoBildaPinpointDriver.class, "POC");
            posGet = new PinpointPosGet(pin);
        } catch (Exception e) {
            telemetry.addData("Config error","Pinpoint not found: " + e);
        }
        try {
            dsl = hardwareMap.get(DistanceSensor.class,"DSL");
            dsr = hardwareMap.get(DistanceSensor.class,"DSR");
            dslC = new CheckedDistanceSensor(dsl,3);
            dsrC = new CheckedDistanceSensor(dsr,3);
        } catch (Exception e) {
            telemetry.addData("Config error","Obstacle dist sensors not found: " + e);
        }
        try {
            turret = hardwareMap.get(DcMotorEx.class,"TR");
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } catch (Exception e) {
            telemetry.addData("Config error","Turret motor not found: " + e);
        }

        try{
            flywheel = hardwareMap.get(DcMotorEx.class, "FW");
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setVelocityPIDFCoefficients(FLYWHEEL_P, 0, 0, FLYWHEEL_F);
        }catch (Exception e){
            telemetry.addData("Config error", "Flywheel not found: " + e);
        }

        try{
            hood = hardwareMap.get(Servo.class, "Hood");
        }catch (Exception e){
            telemetry.addData("Config error", "Hood not found: " + e);
        }

        try{
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
        }catch (Exception e){
            telemetry.addData("Config error", "Lift not found: " + e);
        }

        try{
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        }catch (Exception e){
            telemetry.addData("Config error", "Intake not found: " + e);
        }

        if(!MiscUtils.checkBattery(hardwareMap)) {
            telemetry.addData("OH NO","THE BATTERY IS LOW ON JUICE!!!!!");
            telemetry.speak("The battery is low, please replace it!");
        }
        telemetry.update();
    }
}
