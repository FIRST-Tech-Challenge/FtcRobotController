package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Bot {

    public enum BotState {
        INTAKE, // surgical tubing ready to pick up pixel
        STORAGE_FULL, // either 1 or 2 pixels in storage
        STORAGE_NOT_FULL,
        OUTTAKE, // ready to outtake
    }

    public static Bot instance;

    /*public final Slides slides;
    public final Noodles noodles;
    public final TransferClaw transferClaw;
    Slides, Noodles, and TransferClaw subsystems
     */

    private final DcMotorEx fl, fr, bl, br, susMotor;
    private final Servo tcServo, droneServo_1, droneServo_2, outtakeServo;

    public BotState state = BotState.STORAGE_NOT_FULL;

    public OpMode opMode;

    public BNO055IMU imu;
    public boolean fieldCentricRunMode = false;

    private double imuOffset = 0;
    //verify

    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
        }
        return instance;
    }

    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    private Bot(OpMode opMode) {
        this.opMode = opMode;
        enableAutoBulkRead();
        //what is this
        try {
            this.initializeImus();
            fieldCentricRunMode = false;
        } catch (Exception e) {
            imu = null;
            fieldCentricRunMode = false;

        }


        fl = opMode.hardwareMap.get(DcMotorEx.class, "fl");
        fr = opMode.hardwareMap.get(DcMotorEx.class, "fr");
        bl = opMode.hardwareMap.get(DcMotorEx.class, "bl");
        br = opMode.hardwareMap.get(DcMotorEx.class, "br");
        tcServo = opMode.hardwareMap.get(Servo.class, "tcServo");
        droneServo_1 = opMode.hardwareMap.get(Servo.class, "droneServo_1");;
        droneServo_2= opMode.hardwareMap.get(Servo.class, "droneServo_2");;
        outtakeServo = opMode.hardwareMap.get(Servo.class, "outtakeServo");;
        susMotor = opMode.hardwareMap.get(DcMotorEx.class, "susMotor");

        fl.setMode(RUN_USING_ENCODER);

        //subsystems uwu

        /*this.slides = new Slides(opMode); slides subsystem
        this.noodles = new Noodles(opMode); noodles subsystem
        this.transferClaw = new TransferClaw(opMode); transferClaw subsystem
         */


    }




    /*public void slidesalignjunction() {
        while (horizSlides.getCurrent() > horizSlides.currentthres){
            horizSlides.runManual(0.35);
        }
    }
     */


    //pixelval is an enum, with
    /*public void turretalignjunction() {
        if (PixelDetectionPipeline.junctionVal == PixelDetectionPipeline.JunctionVal.ONLEFT) {
            if (JunctionDetectionPipeline.width > 100) {
                turret.runRawPower(-0.4);
            } else {
                turret.runRawPower(-0.3);
            }
        }
        if (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ONRIGHT) {
            if (JunctionDetectionPipeline.width > 100){
                turret.runRawPower(0.4);
            }
            turret.runRawPower(0.3);
        }
        if (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.NOTDETECTED || JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ATJUNCTION) {
            turret.runRawPower(0);
        }
    }
    public void slowturretalignjunction() {
        if (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ONLEFT) {
            turret.runRawPower(-0.2);
        }
        if (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ONRIGHT) {
            turret.runRawPower(0.2);
        }
        if (JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.NOTDETECTED || JunctionDetectionPipeline.junctionVal == JunctionDetectionPipeline.JunctionVal.ATJUNCTION) {
            turret.runRawPower(0);
        }
    }

     */

    /*
    public void intakeFallen() {
        state = BotState.INTAKE;
        slides.runToBottom();
        arm.fallenintake();
        horizSlides.runToFullIn();
        claw.open();
    }
     */


    public void prepForOuttake() {
        state = BotState.STORAGE_FULL;
        //slides.runToBottom(); code in slides subsystem
    }
    //move to slides

    public void outtake() { // must be combined with bot.slide.run___() in MainTeleOp
        state = BotState.OUTTAKE;
        //slides.runTo(//int arg 1-12, revisit it later); code in slides subsystem
        //transferClaw.open();code in transferClaw subsystem
    }

    public void secure() {
        //pipeline detected pixel, and intake was run
    }
    public void initializeImus() {
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
        resetIMU();
    }


    public void fixMotors(double velocity) {
        fl.setVelocity(-velocity); //invert
        fr.setVelocity(velocity);
        bl.setVelocity(-velocity); // invert
        br.setVelocity(velocity);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                forwardBackSpeed - strafeSpeed - turnSpeed,
                forwardBackSpeed + strafeSpeed + turnSpeed,
                forwardBackSpeed + strafeSpeed - turnSpeed,
                forwardBackSpeed - strafeSpeed + turnSpeed
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.setPower(speeds[0]);
        fr.setPower(speeds[1]);
        bl.setPower(speeds[2]);
        br.setPower(speeds[3]);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed, double heading) {
        double magnitude = Math.sqrt(strafeSpeed * strafeSpeed + forwardBackSpeed * forwardBackSpeed);
        double theta = (Math.atan2(forwardBackSpeed, strafeSpeed) - heading) % (2 * Math.PI);
        double[] speeds = {
                magnitude * Math.sin(theta + Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) - turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta + Math.PI / 4) - turnSpeed
        };

        double maxSpeed = 0;

        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }

        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }

        //        for (int i = 0; i < 4; i++) {
        //            driveTrainMotors[i].set(speeds[i]);
        //        }
        // manually invert the left side

        fl.setPower(speeds[0]);
        fr.setPower(speeds[1]);
        bl.setPower(speeds[2]);
        br.setPower(speeds[3]);
    }

    private void enableAutoBulkRead() {
        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }



    public void resetEncoder() {
        fl.setMode(STOP_AND_RESET_ENCODER);
        fr.setMode(STOP_AND_RESET_ENCODER);
        br.setMode(STOP_AND_RESET_ENCODER);
        bl.setMode(STOP_AND_RESET_ENCODER);
        //slides.resetEncoder(); code this in slides subsystems
        //reset encoder in slides

    }

    public void setImuOffset(double offset) {
        imuOffset += offset;
    }

    public void resetIMU() {
        imuOffset += getIMU();
    }

    public double getIMU() {
        double angle = (imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle - imuOffset) % 360;
        if (angle > 180) {
            angle = angle - 360;
        }
        return angle;
    }

    public void resetProfiler() {
        //slides.resetProfiler(); code in slides subsystem
        //figure this out

    }
}