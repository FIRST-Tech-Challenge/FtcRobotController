package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.network.WifiUtil;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.SystemProperties;

@TeleOp
public class drive extends OpMode {

    private String TESTBOT = "24342-RC";
    private Telemetry.Item telPathDebug = null;
    private MecanumEncoder drive = new MecanumEncoder(this);
    private String wifiSsid = "";

    private DcMotorEx PushMotor;
    private Servo IntakeTilt;
    private Servo IntakeBrush;
    private Servo Bartouch;

    private double IntakeTiltUpMax;
    private double IntakeTiltDownMax;
    private double StartPosition;


    @Override
    public void init() {
        //Continue defining motors
//        PushMotor = hardwareMap.get(DcMotorEx.class, "pushmotor");
//        IntakeTilt = hardwareMap.get(Servo.class, "intaketilt");
//        IntakeBrush = hardwareMap.get(Servo.class, "intakebrush");
        Bartouch = hardwareMap.get(Servo.class, "bartouch");
        //Set direction
//        PushMotor.setDirection(DcMotor.Direction.FORWARD);
//        IntakeTilt.setDirection(Servo.Direction.FORWARD);
//        IntakeBrush.setDirection(Servo.Direction.FORWARD);
//        Bartouch.setDirection(Servo.Direction.FORWARD);
        //Stop and reset
//        PushMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // run once when init is pressed
        wifiSsid = WifiUtil.getConnectedSsid();

        drive.initHardware(hardwareMap, wifiSsid.equals(TESTBOT) ? MecanumEncoder.Bot.TestBot : MecanumEncoder.Bot.CompBot);

        telemetry.clearAll();
        telemetry.setAutoClear(false);
        telPathDebug = telemetry.addData("PathDebug:", "");
    }

    @Override
    public void init_loop() {
        // add stuff here for the init loop
        telPathDebug.setValue(wifiSsid);
        telemetry.update();
    }

    @Override
    public void loop() {
        // runs while in play
        drive.driverInput(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1.0, MecanumEncoder.DriveMode.FieldCentric);

        processBarTouch();
    }
    // Left bumper pushed: Intake grabs sample
    // Right bumper pushed: Intake pushes out sample
    // Both buttons pressed or neither buttons pressed: Intake stops

    //    private void processIntakebrush() {
//        //winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        // Intake in
//        if(gamepad2.left_bumper && !gamepad2.right_bumper) {
//            //winch.setPosition(winch.getPosition()+0.1);
//            IntakeBrush.setDirection(Servo.Direction.FORWARD);
//        }
//        //Intake out
//        if(gamepad2.right_bumper && !gamepad2.left_bumper)  {
//            //winch.setPosition(winch.getPosition()-0.1);
//            IntakeBrush.setDirection(Servo.Direction.REVERSE);
//        }
//        // Intake stop
//        if(!gamepad2.left_bumper && !gamepad2.right_bumper) {
//            IntakeBrush.setPosition(IntakeBrush.getPosition());
//        }
//        // Intake stop
//        if(gamepad2.left_bumper && gamepad2.right_bumper) {
//            IntakeBrush.setPosition(IntakeBrush.getPosition());
//        }
//    }
//    // When dpad_up pressed: robot bucket tilts up
//    // When dpad_down pressed: robot bucket tilts down
//    // Robot bucket can only tilt between a range
//
//    private void processIntaketilt() {
//        //winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        if (gamepad2.dpad_up) {
//            //winch.setPosition(winch.getPosition()+0.1);
//            Double currentPos = IntakeTilt.getPosition();
//            if (currentPos >= 0.0 && currentPos < 1.0) {
//                IntakeTilt.setPosition(currentPos + 0.1);
//            }
//        }
//        if (gamepad2.dpad_down) {
//            //winch.setPosition(winch.getPosition()-0.1);
//            Double currentPos = Bartouch.getPosition();
//            if (currentPos <= 1.0 && currentPos > 0.0) {
//                Bartouch.setPosition(currentPos - 0.1);
//            }
//        }
//    }
    private void processBarTouch() {
        if (gamepad1.dpad_up) {
            Double currentPos = Bartouch.getPosition();
            if (currentPos >= 0.0 || currentPos < 1.0) {
                Bartouch.setPosition(currentPos - 0.01);
                currentPos = Bartouch.getPosition();

            }
        }
        if (gamepad1.dpad_down) {
            Double currentPos = Bartouch.getPosition();
            if (currentPos >= 0.0 || currentPos < 1.0) {
                Bartouch.setPosition(currentPos + 0.01);
                currentPos = Bartouch.getPosition();

            }
        }
    }
}

