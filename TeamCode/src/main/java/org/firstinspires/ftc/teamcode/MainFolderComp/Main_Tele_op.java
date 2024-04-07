/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.MainFolderComp;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the Teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Main Tele-OP", group="A")
//@Disabled
public class Main_Tele_op extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf_drive = null;
    private DcMotor rf_drive = null;
    private DcMotor lb_drive = null;
    private DcMotor rb_drive = null;
    private DcMotor rig = null;
    private DcMotor slide = null;
    private DcMotor elbow = null;

    private
    static final double  COUNTS_PER_MOTOR_REV_SLIDE  = 537.7 ;         // eg: GOBILDA Motor Encoder
    static final double  DRIVE_GEAR_REDUCTION  = 1.0 ;           // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES_SLIDE = 1.40357115168 ; // For figuring circumference
    static final double COUNTS_PER_INCH_SLIDE = (COUNTS_PER_MOTOR_REV_SLIDE * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES_SLIDE * 3.14159265359);

    double SlideTicks = 0;

    Boolean timeoutUp = false;
    Double timerUp = 0.0;
    Boolean timeoutDown = true;
    Double timerDown = 0.0;

    double slideCooldown = 0.5; // in seconds


    private Servo drone = null;

    private Servo l_flick = null;
    private Servo r_flick = null;
    private Servo leftFlap = null;
    private Servo rightFlap = null;
    private CRServo intake = null;

//    private Servo autoarm = null;

    ColorSensor sensorColor;
    ColorSensor sensorColor2;
    DistanceSensor sensorDistance;
    DistanceSensor sensorDistance2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        lb_drive = hardwareMap.get(DcMotor.class, "lb_drive");
        rb_drive = hardwareMap.get(DcMotor.class, "rb_drive");
        rig = hardwareMap.get(DcMotor.class, "rig");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elbow = hardwareMap.get(DcMotor.class, "hexy");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance_2");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance_2");



        drone = hardwareMap.get(Servo.class, "air");

        l_flick = hardwareMap.get(Servo.class, "lflick");
        r_flick = hardwareMap.get(Servo.class, "rflick");
        leftFlap = hardwareMap.get(Servo.class, "lflap");
        rightFlap = hardwareMap.get(Servo.class, "rflap");
        intake = hardwareMap.get(CRServo.class, "intake");

//        autoarm = hardwareMap.get(Servo.class, "autoarm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lf_drive.setDirection(DcMotorEx.Direction.REVERSE);
        lb_drive.setDirection(DcMotorEx.Direction.REVERSE);
        rig.setDirection(DcMotorEx.Direction.REVERSE);
        slide.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(CRServo.Direction.REVERSE);

        rig.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        float hsvValues[] = {0F, 0F, 0F};
        float hsv2Values[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final float values2[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//        double delay = 2.5;
        String foundColor = "null";
//        double lastLeftTime = -(delay) - 0.01; //so that it wouldn't light up at the start
//        double lastRightTime = -(delay) - 0.01; //so that it wouldn't light up at the start

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        boolean rightGate = false, leftGate = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            double lfPower;
            double rfPower;
            double rbPower;
            double lbPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.right_stick_x * 0.75;
            double strafe = gamepad1.left_stick_x;
            double turn = -gamepad1.left_stick_y;
            lfPower = Range.clip(turn + strafe + drive, -1.0, 1.0);
            rfPower = Range.clip(turn - strafe - drive, -1.0, 1.0);
            lbPower = Range.clip(turn - strafe + drive, -1.0, 1.0);
            rbPower = Range.clip(turn + strafe - drive, -1.0, 1.0);

            lf_drive.setPower(lfPower);
            rf_drive.setPower(rfPower);
            lb_drive.setPower(lbPower);
            rb_drive.setPower(rbPower);

//            if (gamepad1.x) {
//                drone.setPosition(1);
//            } else {
//                drone.setPosition(0);
//            }

            if (gamepad1.dpad_up) {
                rig.setPower(1);
            } else if (gamepad1.dpad_down) {
                rig.setPower(-1);
            } else {
                rig.setPower(0);
            }

            while (gamepad1.y) {
                slide.setPower(0.5);
//                timeoutUp = true;
//                timerUp = runtime.seconds();
//                telemetry.update();
            }

            while (gamepad1.a) {
                slide.setPower(-0.5);

//                timeoutDown = true;
//                timerDown = runtime.seconds();
//                telemetry.update();
            }
            slide.setPower(0);



            if (gamepad1.left_stick_button) {
                intake.setPower(1);

            } else {
                intake.setPower(0);
            }

            if(gamepad1.right_bumper){
                l_flick.setPosition(0);
                r_flick.setPosition(0.68);
            }
            if(gamepad1.left_bumper){
                l_flick.setPosition(0.59);
                r_flick.setPosition(0);
            }

            if(gamepad1.x){
                leftFlap.setPosition(0.4);
                leftGate = true;
            }
            else{
                leftFlap.setPosition(1);
                leftGate = false;
            }
            if(gamepad1.b){
                rightFlap.setPosition(0.6);
                rightGate = true;
            }
            else{
                rightFlap.setPosition(0);
                rightGate = false;
            }


//            double elbowPower = gamepad2.left_stick_y;
//            elbow.setPower(elbowPower);
//            if ((runtime.seconds() - timerUp) >= slideCooldown) {
//                timeoutUp = false;
//            }
//            if ((runtime.seconds() - timerDown) >= slideCooldown) {
//                timeoutDown = false;
//            }
            while (gamepad1.dpad_left) {
                elbow.setPower(0.5);
            }
            while (gamepad1.dpad_right) {
                elbow.setPower(-0.5);
            }

//            autoarm.setPosition(1);

            telemetry.addData("CurrentSlideTicks:", SlideTicks);

            double x = runtime.milliseconds();
//            telemetry.addData("Red", sensorColor.red());
//            telemetry.addData("Green", sensorColor.green());
//            telemetry.addData("Blue", sensorColor.blue());
//            telemetry.addData("Red2", sensorColor2.red());
//            telemetry.addData("Green2", sensorColor2.green());
////            telemetry.addData("Blue2", sensorColor2.blue());
//            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
//                    (int) (sensorColor.green() * SCALE_FACTOR),
//                    (int) (sensorColor.blue() * SCALE_FACTOR),
//                    hsvValues);
//            Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
//                    (int) (sensorColor2.green() * SCALE_FACTOR),
//                    (int) (sensorColor2.blue() * SCALE_FACTOR),
//                    hsv2Values);


            telemetry.addData("time delay", runtime.milliseconds() - x);

            String leftdetectedColor = "Gate open or Unknown";
            String rightdetectedColor = "Gate open or Unknown";
//            double hue = rgb_to_hsv(sensorColor.red(), sensorColor.green(), sensorColor.blue());
//            double hue2 = rgb_to_hsv(sensorColor2.red(), sensorColor2.green(), sensorColor2.blue());
//            double hue = hsvValues[0];
//            double hue2 = hsv2Values[0];
//            if(!leftGate) {
//                if (hue >= 0 && hue < 60 || hue > 360) {
//                    leftdetectedColor = "Red";
//                } else if (hue >= 60 && hue < 120) {
//                    leftdetectedColor = "Yellow";
//                } else if (hue >= 120 && hue < 150) {
//                    leftdetectedColor = "Green";
//                } else if (hue >= 210 && hue < 300) {
//                    leftdetectedColor = "Blue";
//                } else if (hue >= 180 && hue < 210) {
//                    leftdetectedColor = "Purple";
//                } else if (hue > 150 && hue < 180) {
//                    leftdetectedColor = "White";
//                }
//            }
//
//            telemetry.addData("Hues", hue + " " + hue2);
//            if(!rightGate) {
//                if (hue2 >= 0 && hue2 < 60 || hue2 > 360) {
//                    rightdetectedColor = "Red";
//                } else if (hue2 >= 60 && hue2 < 120) {
//                    rightdetectedColor = "Yellow";
//                } else if (hue2 >= 120 && hue2 < 150) {
//                    rightdetectedColor = "Green";
//                } else if (hue2 >= 210 && hue2 < 300) {
//                    rightdetectedColor = "Blue";
//                } else if (hue2 >= 180 && hue2 < 210) {
//                    rightdetectedColor = "Purple";
//                } else if (hue2 > 150 && hue2 < 180) {
//                    rightdetectedColor = "White";
//                }
//            }
//            boolean pixelInLeft = true, pixelInRight = true;
//            if(sensorDistance.getDistance(DistanceUnit.INCH) > 1){
//                leftdetectedColor = "No Pixel";
//                pixelInLeft = false;
//            }
//            if(sensorDistance2.getDistance(DistanceUnit.INCH) > 1){
//                rightdetectedColor = "No Pixel";
//                pixelInRight = false;
//            }

//            telemetry.addData("Left Color", leftdetectedColor);
//            telemetry.addData("Right Color", rightdetectedColor);
//            telemetry.addData("Left Pixel", pixelInLeft);
//            telemetry.addData("Right Pixel", pixelInRight);
//            telemetry.update();

        }
    }

    static double rgb_to_hsv(double r, double g, double b)
    {

        // R, G, B values are divided by 255
        // to change the range from 0..255 to 0..1
        r = r / 255.0;
        g = g / 255.0;
        b = b / 255.0;

        // h, s, v = hue, saturation, value
        double cmax = Math.max(r, Math.max(g, b)); // maximum of r, g, b
        double cmin = Math.min(r, Math.min(g, b)); // minimum of r, g, b
        double diff = cmax - cmin; // diff of cmax and cmin.
        double h = -1, s = -1;

        // if cmax and cmax are equal then h = 0
        if (cmax == cmin)
            h = 0;

            // if cmax equal r then compute h
        else if (cmax == r)
            h = (60 * ((g - b) / diff) + 360) % 360;

            // if cmax equal g then compute h
        else if (cmax == g)
            h = (60 * ((b - r) / diff) + 120) % 360;

            // if cmax equal b then compute h
        else if (cmax == b)
            h = (60 * ((r - g) / diff) + 240) % 360;

        // if cmax equal zero
        if (cmax == 0)
            s = 0;
        else
            s = (diff / cmax) * 100;

        // compute v
        double v = cmax * 100;
        return h;

    }

    public void encoderSlideUpInches(double Inches) {
        double TicksToMove = Inches * COUNTS_PER_INCH_SLIDE;
        slide.setTargetPosition(((int) TicksToMove));
        slide.setPower(0.5);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long) (Inches * 2 * 25.4));
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setPower(0);
    }
    public void encoderSlideDownInches(double inches) {
        double TicksToMove = inches * COUNTS_PER_INCH_SLIDE;
        slide.setTargetPosition(-((int) TicksToMove));
        slide.setPower(0.5);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long) (inches * 2 * 25.4));
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setPower(0);
    }
}