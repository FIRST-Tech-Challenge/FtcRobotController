package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
//import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public abstract class Taco_FF_Super_Class extends LinearOpMode {


   // public AndroidSoundPool androidSoundPool;

    public DcMotor Duck;
    public DcMotor Left_Front;
    public DcMotor Left_Rear;
    public DcMotor Right_Front;
    public DcMotor Right_Rear;
    public DcMotor Arm;

    public CRServo Roller;
    public CRServo Wheel;

    public RevBlinkinLedDriver Blinkyboi;

    public boolean Delay = false;
    public boolean far = true;
    public boolean spin = false;
    public boolean park = false;
    public double start_time = 0;
    public int arm_target = (0);
    static int times_run = 0;
    static boolean blue = false;

    public void initialization(boolean autonomous) {

        if (autonomous) {
            initDuck();
            initArm();
            initRight_Rear();
            initLeft_Front();
            initLeft_Rear();
            initRight_Front();
            initWheel();
            initRoller();
            initBlink();
            Left_Front.setDirection(DcMotorSimple.Direction.FORWARD);
            Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
            Left_Rear.setDirection(DcMotorSimple.Direction.FORWARD);
            Right_Rear.setDirection(DcMotorSimple.Direction.REVERSE);
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            initBlink();
            initDuck();
            initArm();
            initRight_Rear();
            initLeft_Front();
            initLeft_Rear();
            initRight_Front();
            initWheel();
            initRoller();
        }
        telemetry.addLine("Finished init");
        telemetry.update();
    }

    public void initDuck() {
        Duck = hardwareMap.dcMotor.get("Duck");
        Duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initLeft_Front() {
        Left_Front = hardwareMap.dcMotor.get("Left_Front");
        Left_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initRight_Front() {
        Right_Front = hardwareMap.dcMotor.get("Right_Front");
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initLeft_Rear() {
        Left_Rear = hardwareMap.dcMotor.get("Left_Rear");
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setDirection(DcMotorSimple.Direction.REVERSE);
        Left_Rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initRight_Rear() {
        Right_Rear = hardwareMap.dcMotor.get("Right_Rear");
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initArm() {
        Arm = hardwareMap.dcMotor.get("Arm");
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initBlink(){
        Blinkyboi = hardwareMap.get(RevBlinkinLedDriver.class, "BlinkyBoi");
    }

    public void initWheel() {
        Wheel = hardwareMap.get(CRServo.class, "Wheel");
    }

    public void initRoller() {
        Roller = hardwareMap.get(CRServo.class, "Roller");
    }


    public void go_same(int all, double power, int wait) {
        if (!isStopRequested()) {
            Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right_Front.setTargetPosition(all);
            Right_Rear.setTargetPosition(all);
            Left_Rear.setTargetPosition(all);
            Left_Front.setTargetPosition(all);
            Right_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Front.setPower(power);
            Right_Rear.setPower(power);
            Left_Front.setPower(power);
            Left_Rear.setPower(power);
            sleep(wait);
        }
    }


    public void go_diff(int lf, int lr, int rf, int rr, double power, int wait) {
        if (!isStopRequested()) {
            Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right_Front.setTargetPosition(rf);
            Right_Rear.setTargetPosition(rr);
            Left_Rear.setTargetPosition(lr);
            Left_Front.setTargetPosition(lf);
            Right_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Front.setPower(power);
            Right_Rear.setPower(power);
            Left_Front.setPower(power);
            Left_Rear.setPower(power);
            sleep(wait);
        }
    }

    public void drive() {
        if (!isStopRequested()) {
            if (gamepad1.right_trigger > .3) {
                Right_Front.setPower(gamepad1.right_stick_y / -2);
                Right_Rear.setPower(gamepad1.right_stick_y / -2);
                Left_Front.setPower(gamepad1.left_stick_y / -2);
                Left_Rear.setPower(gamepad1.left_stick_y / -2);
            } else {
                Right_Front.setPower(gamepad1.right_stick_y/-1);
                Right_Rear.setPower(gamepad1.right_stick_y/-1);
                Left_Front.setPower(gamepad1.left_stick_y/-1);
                Left_Rear.setPower(gamepad1.left_stick_y/-1);
            }
        }
    }

    public void lift() {
        if (!isStopRequested()) {
            if (gamepad2.y) {
                arm_target = 600;
                Arm.setTargetPosition(arm_target);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(-.6);
            } else if (gamepad2.a) {
                arm_target = 0;
                Arm.setTargetPosition(arm_target);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(-.3);
            } else if (gamepad2.b) {
                arm_target = 225;
                Arm.setTargetPosition(arm_target);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(-.4);
            } else if (gamepad1.b){
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        }
    }

    public void color(){
        if (!isStopRequested()){
            if (gamepad1.left_bumper){
                blue = false;
            } else if (gamepad1.right_bumper){
                blue = true;
            }

        }
    }

    public void time(){
        if (!isStopRequested()) {
            if ((getRuntime() - start_time) < 80) {
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(88));
            } else if ((getRuntime() - start_time)  > 120) {
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(44));
            } else if ((getRuntime() - start_time)  > 115) {
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(80));
            } else if ((getRuntime() - start_time)  > 90) {
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(82));
            } else if ((getRuntime() - start_time)  > 80) {
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(85));
            }
        }
    }

        public void intake(){
            if (!isStopRequested()) {
                if (gamepad2.right_stick_y > .3) {
                    ((PwmControl)Wheel).setPwmEnable();
                    ((PwmControl)Roller).setPwmEnable();
                    Wheel.setPower(1);
                    Roller.setPower(-1);
                } else if (gamepad2.right_stick_y < -.3) {
                    ((PwmControl)Wheel).setPwmEnable();
                    ((PwmControl)Roller).setPwmEnable();
                    Wheel.setPower(-.1);
                    Roller.setPower(.1);
                } else {
                    ((PwmControl)Wheel).setPwmDisable();
                    ((PwmControl)Roller).setPwmDisable();
                }
            }
        }

        public void duck () {
            if (!isStopRequested()) {
                if (blue) {
                    if (gamepad2.right_trigger > .3) {
                        Duck.setPower(1);
                    } else if (gamepad2.left_trigger > .3) {
                        Duck.setPower(.3);
                    } else {
                        Duck.setPower(0);
                    }
                } else {
                    if (gamepad2.right_trigger > .3) {
                        Duck.setPower(-1);
                    } else if (gamepad2.left_trigger > .3) {
                        Duck.setPower(-.3);
                    } else {
                        Duck.setPower(0);
                    }
                }
            }
    }

    public void Spot() {
        while (!isStarted()) {
            if (gamepad1.x) {
                blue = true;
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(42));
            } else if (gamepad1.b) {
                blue = false;
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(41));
            } else if (gamepad1.a) {
                far = true;
            } else if (gamepad1.y) {
                far = false;
            } else if (gamepad1.left_trigger > 0.3) {
                Delay = false;
            } else if (gamepad1.right_trigger > 0.3) {
                Delay = true;
            } else if (gamepad1.left_bumper) {
                spin = false;
            } else if (gamepad1.right_bumper) {
                spin = true;
            } else if (gamepad1.options){
                park = true;
            } else if (gamepad1.share) {
                park = false;
            }
            telemetry.addData(">", "Please set up auto...");
            if (blue){
                Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(42));
                telemetry.addData("Alliance", "Blue");
                if (park){
                    telemetry.addData("Park in box", "True");
                } else {
                    telemetry.addData("Park in box", "False");
                }
            } else {
                telemetry.addData("Alliance", "Red");
            }
            if (far){
                telemetry.addData("Side", "Ducks");
                telemetry.addData("Spin ducks", spin);
            } else {
                telemetry.addData("Side", "Shared Hub");
            }
            telemetry.addData("Delay", Delay);
            telemetry.update();
        }
    }
}