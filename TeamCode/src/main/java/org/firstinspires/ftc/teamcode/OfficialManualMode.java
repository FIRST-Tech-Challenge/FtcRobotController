package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;

@TeleOp(name = "Official Manual Mode", group = "Match")
public class OfficialManualMode extends LinearOpMode {
    private DcMotor _fl, _fr, _rl, _rr;
    private Servo _grip, _platform, _elbow, _shoulder;
    private double _armPosition = 0, _leftTrigger = 0, _rightTrigger = 0;
    private boolean logMode = false;
    private ArrayList<String> logArray = new ArrayList<>();
    private boolean firstTime = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        _fl = hardwareMap.dcMotor.get("frontLeft");
        _fr = hardwareMap.dcMotor.get("frontRight");
        _rl = hardwareMap.dcMotor.get("rearLeft");
        _rr = hardwareMap.dcMotor.get("rearRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        _fr.setDirection(DcMotorSimple.Direction.REVERSE);
        _rr.setDirection(DcMotorSimple.Direction.REVERSE);

        _platform = hardwareMap.get(Servo.class, "platform");
        _grip = hardwareMap.get(Servo.class, "grip");
        _elbow = hardwareMap.get(Servo.class, "elbow");
        _shoulder = hardwareMap.get(Servo.class, "shoulder");
       // _elbow.scaleRange(0.2,0.8);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (firstTime) {
                firstTime = false;
                resetToPresetPosition(0);
            }
            controlArm();
            controlWheels();
        }
    }

    private void controlWheels() {
        double y = gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x * 0.5;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        double speedmultiplier = 1;
        if (gamepad1.left_trigger > 0) {
            speedmultiplier = 2;
        } else if (gamepad1.right_trigger > 0) {
            speedmultiplier = 0.5;
        } else {
            speedmultiplier = 1;
        }
        _fl.setPower(frontLeftPower * speedmultiplier);
        _rl.setPower(backLeftPower * speedmultiplier);
        _fr.setPower(frontRightPower * speedmultiplier);
        _rr.setPower(backRightPower * speedmultiplier);
        telemetry.addData("FLSpd", frontLeftPower * speedmultiplier);
        telemetry.addData("RRSpd", backRightPower * speedmultiplier);
        telemetry.update();
    }



        private void controlArm(){
            double _elpos = _elbow.getPosition();
            double _armPosition = _shoulder.getPosition();
            _leftTrigger = gamepad2.left_trigger;
            _rightTrigger = gamepad2.right_trigger;

        if (gamepad2.share) {
            replayLogAction(logArray);
            return;
        }
        if (gamepad2.start) {
            setLogMode();
        }
        if (gamepad2.left_bumper) {
            logAction("left_bumper");
            _platform.setPosition(_platform.getPosition() - 0.05);
        }
        if (gamepad2.right_bumper) {
            logAction("right_bumper");
            _platform.setPosition(_platform.getPosition() + 0.05);
        }

        if (gamepad2.x) {
            logAction("x");
            telemetry.addData("WEEWOO IT GO UP SHOLDER", 0.7);
            _elbow.setPosition(0.7);
            _shoulder.setPosition(0.7);
        }

        if (gamepad2.a) {
            telemetry.addData("I SWEAR SHOUOLDER MOVE", 0);
          logAction("a");
            _shoulder.setPosition(1);
        }
        if (gamepad2.b) {
            logAction("b");
            _elbow.setPosition(0);
        }
        if (gamepad2.y) {
            logAction("y");
            _elbow.setPosition(1);
        }
        if (gamepad2.left_stick_button) {
            logAction("left_stick_button");
            _grip.setPosition(1);
        }
        if (gamepad2.right_stick_button) {
            logAction("right_stick_button");
            _grip.setPosition(0);
        }

        if (gamepad2.left_trigger > 0 && _rightTrigger == 0 && _elpos < 1.0) {
            logAction("left_trigger");
            _elbow.setPosition(_elbow.getPosition() + 0.1); // or elbow.setPosition(elpos + 0.05);
        }
        if (gamepad2.right_trigger > 0 && _leftTrigger == 0 && _elpos > 0.2) {
            logAction("right_trigger");
            _elbow.setPosition(_elbow.getPosition() - 0.1);
        }

        //_elbow.setPosition(_armPosition);

        double intx = gamepad2.left_stick_x;
        double inty = gamepad2.left_stick_y;
        double otherY = gamepad2.right_stick_y;
        telemetry.addData("This is our left stick X and this is updated", intx);
        //telemetry.update();
        double shoulderDir = _shoulder.getPosition() + inty * 0.2;
        double prefDir = _platform.getPosition() + intx*0.2;
        double prefDir2 = _elbow.getPosition() + otherY *0.2;
        if (prefDir > 1) {
            prefDir = 1;
        } else if (prefDir < 1) {
            prefDir = 0;
        }
        if(shoulderDir > 1) {
            shoulderDir = 1;
        }else if (shoulderDir < 1) {
            shoulderDir = 0;
        }
        if(prefDir2 > 1){
            prefDir2 = 1;
        }else if (prefDir2 < 1) {
            prefDir2 = 0;
        }
      //_platform.setPosition(prefDir);
     // _elbow.setPosition(prefDir2);
      //_shoulder.setPosition(shoulderDir);
        telemetry.addData("Platform que quiere este", prefDir);
    }

//    private void controlArmBak(){
//        double _elpos = _elbow.getPosition();
//        double _armPosition = _shoulder.getPosition();
//        _leftTrigger = gamepad2.left_trigger;
//        _rightTrigger = gamepad2.right_trigger;
//
//        if (gamepad2.left_bumper) {
//            _platform.setPosition(1);
//        }
//        else if (gamepad2.right_bumper)
//            _platform.setPosition(0);
//        else if (gamepad2.x) {
//            _elbow.setPosition(0);
//            _shoulder.setPosition(0.5);
//        }
//        else if (gamepad2.a)
//            _shoulder.setPosition(0);
//        else if (gamepad2.b)
//            _elbow.setPosition(0.5);
//        else if (gamepad2.y)
//            _elbow.setPosition(0);
//        else if (gamepad2.left_stick_button)
//            _grip.setPosition(1);
//        else if (gamepad2.right_stick_button)
//            _grip.setPosition(0);
//
//        else if (gamepad2.left_trigger > 0 && _rightTrigger == 0 && _elpos < 1.0)
//            _elbow.setPosition(_elbow.getPosition() + 0.05); // or elbow.setPosition(elpos + 0.05);
//        else if (gamepad2.right_trigger > 0 && _leftTrigger == 0 && _elpos > 0)
//            _elbow.setPosition(_elbow.getPosition() - 0.05);
//
//        //_elbow.setPosition(_armPosition);
//
//        double intx = gamepad2.left_stick_x;
//        double inty = gamepad2.left_stick_y;
//        double otherY = gamepad2.right_stick_y;
//        telemetry.addData("This is our left stick X and this is updated", intx);
//        //telemetry.update();
//        double shoulderDir = _shoulder.getPosition() + inty * 0.2;
//        double prefDir = _platform.getPosition() + intx*0.2;
//        double prefDir2 = _elbow.getPosition() + otherY *0.2;
//        if (prefDir > 1) {
//            prefDir = 1;
//        } else if (prefDir < 1) {
//            prefDir = 0;
//        }
//        if(shoulderDir > 1) {
//            shoulderDir = 1;
//        }else if (shoulderDir < 1) {
//            shoulderDir = 0;
//        }
//        if(prefDir2 > 1){
//            prefDir2 = 1;
//        }else if (prefDir2 < 1) {
//            prefDir2 = 0;
//        }
       // _platform.setPosition(prefDir);
        //_elbow.setPosition(prefDir2);
        //_shoulder.setPosition(shoulderDir);
        //telemetry.addData("Platform que quiere este", prefDir);

//
    private void playAction(String actionName) {
        if (actionName.equals("left_bumper")) {
            _platform.setPosition(1);
        }
        else if (actionName.equals("right_bumper")) {
            _platform.setPosition(0);
        }
        else if (actionName.equals("x")) {
            _elbow.setPosition(0);
            _shoulder.setPosition(0.5);
        }
        else if (actionName.equals("a")) {
            _shoulder.setPosition(0);
        }
        else if (actionName.equals("b")) {
            _elbow.setPosition(0.5);
        }
        else if (actionName.equals("y")) {
            _elbow.setPosition(0);
        }
        else if (actionName.equals("left_stick_button")) {
            _grip.setPosition(1);
        }
        else if (actionName.equals("right_stick_button")) {
            _grip.setPosition(0);
        }
        else if (actionName.equals("left_trigger")) {
            _elbow.setPosition(_elbow.getPosition() + 0.05); // or elbow.setPosition(elpos + 0.05);
        }
        else if (actionName.equals("right_trigger")) {
            _elbow.setPosition(_elbow.getPosition() - 0.05);
        }

    }

    private void resetToPresetPosition(int resetMode) {
        if (resetMode == 0) {
            // center the control arms
            _shoulder.setPosition(0);
            _platform.setPosition(0.5);
            _elbow.setPosition(0.5);
        }
    }
//
    private void setLogMode() {
        if (logMode == false) {
            logMode = true;
            logArray.clear();
            System.out.println("Start log...");
            telemetry.addData("Start log...", 1);
        }
        else {
            logMode = false;
            // print out the moves / buttons pressed since log start;
            System.out.println("Operations: " + logArray);
            telemetry.addData("Operations: ", logArray);
        }
    }

    private void logAction(String s) {
        if (logMode == true) {
            logArray.add(s);
        }
        telemetry.addData("logAction", s);
    }
//
    private void replayLogAction(ArrayList<String> list) {
        for (int index = 0; index < list.size(); index++) {
            playAction(list.get(index));
        }
    }

  }


