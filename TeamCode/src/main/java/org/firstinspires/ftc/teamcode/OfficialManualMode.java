package org.firstinspires.ftc.teamcode;

import android.view.ViewDebug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "Official Manual Mode", group = "Match")
public class OfficialManualMode extends LinearOpMode {
    private DcMotor _fl, _fr, _rl, _rr;
    private Servo _grip, _platform, _elbow, _shoulder;
    private double _armPosition = 0, _leftTrigger = 0, _rightTrigger = 0;
    private boolean logMode = false;
    private ArrayList<String> logArray = new ArrayList<>();
    private boolean firstTime = true;
    private ElapsedTime recentActionTime = new ElapsedTime();
    public double perStepSize = 0.01;
    public int minTimeOfTwoOperations = 50; //milliseconds, 0.05 second

    // "wheel_forward @10 @0.5", wheel_back 10inch and speed is 0.5
    // wheel_left/wheel_right/wheel_back
    // platform and shoulder elbow remain still, position / direction not changed

    // "time_wheel_left @3500 @0.2" : go straight left for 3500 milliseconds at speed 0.2
    // "time_wheel_right @5000 @0.3"
    // "time_wheel_forward @2000 @0.3" "time_wheel_back @3000 @0.3"
    // platform / shoulder / elbow position not changed with "time_wheel_" action

    // "wheel_turn_left @8 @0.3" turn left about 90 degree at speed 0.3
    // "wheel_turn_left @16 @0.2" turn about 180 degree at speed 0.2
    // platform / shoulder / elbow direction will change at same time

    // "platform_left @10" : platform turn left 10 times the perStepSize
    // "platform_right @20

    // "shoulder_up @10" "shoulder_down @20"
    //

    // "elbow_up @10" "elbow_down @20"
    //

    // "grip_max" "grip_min" "grip_open @10" "grip_close @10"
    //

    public ArrayList<String> presetActionsPad1A = new ArrayList<String>(Arrays.asList(
            "wheel_forward @15 @0.2",
            "sleep @500",
            "wheel_left @20 @0.2",
            "sleep @500",
            "platform_left @20",
            "shoulder_up @30",
            "elbow_up @40",
            "sleep @500",
            "grip_max",
            "sleep @1000",
            "grip_min",
            "elbow_down @40",
            "shoulder_down @20",
            "platform_right @20",
            "wheel_right @40 @0.2",
            "sleep @500",
            "wheel_back @20 @0.2",
            "sleep @1000"
    ));

    public ArrayList<String> presetActionsPad1B = new ArrayList<String>(Arrays.asList(
            "wheel_turn_right @12 @0.3",
            "sleep @1000",
            "wheel_back @12 @0.3",
            "sleep @1000",
            "wheel_turn_right @10 @0.3",
            "sleep @1000",
            "wheel_back @12 @0.3",
            "sleep @1000",
            "wheel_turn_right @10 @0.3",
            "sleep @1000",
            "wheel_back @12 @0.3",
            "sleep @1000",
            "wheel_turn_right @10 @0.3"
    ));
    public ArrayList<String> presetActionsPad2X = new ArrayList<String>(Arrays.asList(
            "wheel_turn_left @14 @0.2",
            "sleep @2000",
            "wheel_forward @10 @0.2",
            "sleep @1000",
            "wheel_turn_left @14 @0.2",
            "sleep @1000",
            "wheel_forward @10 @0.3",
            "sleep @1000",
            "wheel_turn_left @14 @0.2",
            "sleep @1000",
            "wheel_forward @10 @0.3",
            "sleep @1000",
            "wheel_turn_left @14 @0.2",
            "sleep @1000"
    ));
    public ArrayList<String> presetActionsPad2Y = new ArrayList<String>(Arrays.asList(
            "wheel_forward @10 @0.2",
            "sleep @1000",
            "wheel_right @10 @0.2",
            "sleep @1000",
            "wheel_back @10 @0.2",
            "sleep @1000",
            "wheel_left @10 @0.2",
            "sleep @1000"
    ));
    public ArrayList<String> presetActionsPad2A = new ArrayList<String>(Arrays.asList(
            "time_wheel_left @3000 @0.3",
            "sleep @1000",
            "time_wheel_forward @3000 @0.3",
            "sleep @1000",
            "time_wheel_right @3000 @0.3",
            "sleep @1000",
            "time_wheel_back @3000 @0.3"
    ));
    public ArrayList<String> presetActionsPad2B = new ArrayList<String>(Arrays.asList(
            "wheel_turn_left @15.5 @0.2"
    ));
    // 1X and 1Y currently used for log, and these 2 are not used now
    public ArrayList<String> presetActionsPad1X = new ArrayList<String>(Arrays.asList(
            "wheel_turn_left @15.4 @0.3"
    ));
    public ArrayList<String> presetActionsPad1Y = new ArrayList<String>(Arrays.asList(
            "wheel_turn_right @15.6 @0.3"
    ));

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

        // RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER
        _fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _rl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  " %7d :%7d :%7d :%7d",
                _fl.getCurrentPosition(), _fr.getCurrentPosition(), _rl.getCurrentPosition(), _rr.getCurrentPosition());
        //telemetry.addData("wheel device name", _fl.getDeviceName());
        //telemetry.addData("wheel manufacturer", _fl.getManufacturer());
        //telemetry.update();

        _platform = hardwareMap.get(Servo.class, "platform");
        _grip = hardwareMap.get(Servo.class, "grip");
        _elbow = hardwareMap.get(Servo.class, "elbow");
        _shoulder = hardwareMap.get(Servo.class, "shoulder");

        //_platform.scaleRange(0.0, 1.0);
        //_shoulder.scaleRange(0.0, 1.0);
        //_elbow.scaleRange(0.0, 1.0);
        //_grip.scaleRange(0.0, 1.0);

        telemetry.addData("_grip device name", _grip.getDeviceName());
        telemetry.addData("_grip manufacturer", _grip.getManufacturer());
        telemetry.update();

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
        if (gamepad1.a) {
            replayActions(presetActionsPad1A);
            return;
        }
        if (gamepad1.b) {
            replayActions(presetActionsPad1B);
            return;
        }
        if (gamepad1.x) {
            setLogMode(true);
            return;
        }
        if (gamepad1.y) {
            setLogMode(false);
            return;
        }
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
        //telemetry.addData("FLSpd", frontLeftPower * speedmultiplier);
        //telemetry.addData("RRSpd", backRightPower * speedmultiplier);
        //telemetry.update();
    }



    private void controlArm() {
        if (gamepad2.x) {
            replayActions(presetActionsPad2X);
            return;
        }
        if (gamepad2.a) {
            replayActions(presetActionsPad2A);
            return;
        }
        if (gamepad2.b) {
            replayActions(presetActionsPad2B);
            return;
        }
        if (gamepad2.y) {
            replayActions(presetActionsPad2Y);
            return;
        }

        if (gamepad1.dpad_left) {
            playAction("grip_open", true);
        }
        if (gamepad1.dpad_right) {
            playAction("grip_close", true);
        }
        if (gamepad2.left_bumper) {
            playAction("left_bumper", true);
        }
        if (gamepad2.right_bumper) {
            playAction("right_bumper", true);
        }
        if (gamepad2.dpad_up) {
            playAction("dpad_up", true);
        }
        if (gamepad2.dpad_down) {
            playAction("dpad_down", true);
        }
        if (gamepad2.dpad_left) {
            playAction("dpad_left", true);
        }
        if (gamepad2.dpad_right) {
            playAction("dpad_right", true);
        }
        if (gamepad2.left_stick_button) {
            playAction("left_stick_button", true);
            //_grip.setPosition(1);
        }
        if (gamepad2.right_stick_button) {
            playAction("right_stick_button", true);
            //_grip.setPosition(0);
        }
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {
            playAction("left_trigger", true);
        }
        if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) {
            playAction("right_trigger", true);
        }

        double intx = gamepad2.left_stick_x;
        double inty = gamepad2.left_stick_y;
        double otherY = gamepad2.right_stick_y;
        //telemetry.addData("This is our left stick X and this is updated", intx);
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
        //telemetry.addData("Platform que quiere este", prefDir);
    }

    private boolean shoulderMoved = false;
    private void playAction(String actionName, boolean ignoreRecent) {
        if (ignoreRecent) {
            if (recentActionTime.milliseconds() < minTimeOfTwoOperations) {
                // too close to last action, ignore it
                return;
            }
            else {
                recentActionTime.reset();
            }
        }
        logAction(actionName);
        if (actionName.equals("dpad_left") || actionName.equals("platform_left")) {
            if (_platform.getPosition() <= (1 - perStepSize))
                _platform.setPosition(_platform.getPosition() + perStepSize);
        }
        else if (actionName.equals("dpad_right") || actionName.equals("platform_right")) {
            if (_platform.getPosition() >= perStepSize)
                _platform.setPosition(_platform.getPosition() - perStepSize);
        }
        else if (actionName.equals("dpad_up") || actionName.equals("shoulder_up")) {
            if (_shoulder.getPosition() >= perStepSize) {
                _shoulder.setPosition(_shoulder.getPosition() - perStepSize);
            }
            /*
            if (shoulderMoved) {
                shoulderMoved = false;
                if (_shoulder.getDirection() == Servo.Direction.FORWARD)
                    _shoulder.setDirection(Servo.Direction.REVERSE);
                else
                    _shoulder.setDirection(Servo.Direction.FORWARD);
                _shoulder.setPosition(0);
            }
            else {
                if (_shoulder.getPosition() >= perStepSize) {
                    _shoulder.setPosition(_shoulder.getPosition() - perStepSize);
                    shoulderMoved = true;
                }
            }
            */
        }
        else if (actionName.equals("dpad_down") || actionName.equals("shoulder_down")) {
            if (_shoulder.getPosition() <= (1 - perStepSize)) {
                _shoulder.setPosition(_shoulder.getPosition() + perStepSize);
            }
            /*
            if (shoulderMoved) {
                if (_shoulder.getDirection() == Servo.Direction.FORWARD)
                    _shoulder.setDirection(Servo.Direction.REVERSE);
                else
                    _shoulder.setDirection(Servo.Direction.FORWARD);
                shoulderMoved = false;
                _shoulder.setPosition(_shoulder.getPosition());
            }
            else {
                if (_shoulder.getPosition() <= (1 - perStepSize)) {
                    _shoulder.setPosition(_shoulder.getPosition() + perStepSize);
                    shoulderMoved = true;
                }
            }
            */
        }
        else if (actionName.equals("grip_open")) {
            if (_grip.getPosition() <= (1 - perStepSize))
                _grip.setPosition(_grip.getPosition() + perStepSize);
            //_grip.setPosition(1);
        }
        else if (actionName.equals("left_stick_button") || actionName.equals("grip_max")) {
            _grip.setPosition(0.8);
        }
        else if (actionName.equals("grip_close")) {
            if (_grip.getPosition() >= perStepSize)
                _grip.setPosition(_grip.getPosition() - perStepSize);
            //_grip.setPosition(0);
        }
        else if (actionName.equals("right_stick_button") || actionName.equals("grip_min")) {
            _grip.setPosition(0.1);
        }
        else if (actionName.equals("left_trigger") || actionName.equals("elbow_up")) {
            if (_elbow.getPosition() > perStepSize) {
                _elbow.setPosition(_elbow.getPosition() - perStepSize);
            }
        }
        else if (actionName.equals("right_trigger") || actionName.equals("elbow_down")) {
            if (_elbow.getPosition() < (1 - perStepSize)) {
                _elbow.setPosition(_elbow.getPosition() + perStepSize);
            }
        }

    }

    private void resetToPresetPosition(int presetMode) {
        telemetry.addData("Preset position", presetMode);
        if (presetMode == 0) {
            // center the control arms
            _platform.setPosition(0.41);
            _shoulder.setPosition(0.5);
            _elbow.setPosition(0.75);
        }
    }
//
    private void setLogMode(boolean mode) {
        if (logMode == mode)
            return;
        if (mode == true) {
            logMode = true;
            logArray.clear();
            //System.out.println("Start log...");
            telemetry.addData("Start log...", logArray.size());
            telemetry.update();
        }
        else if (mode == false) {
            telemetry.addData("Stop log...", logArray.size());
            logMode = false;
            // print out the moves / buttons pressed since log start;
            //System.out.println("Operations: " + logArray);
            for (int index = 0; index < logArray.size(); index++) {
                String s = Integer.toString(index);
                s += " ";
                s += logArray.get(index);
                telemetry.addData("Operations: ", s);
                telemetry.log().add(s);
            }
            //telemetry.addData("Operations: ", logArray);
            telemetry.update();
        }
    }

    private void logAction(String s) {
        if (logMode == true) {
            logArray.add(s);
        }
        telemetry.addData("logAction", s);
        telemetry.addData("_platform ", _platform.getPosition());
        //telemetry.addData("_platform ", _platform.getDirection());
        telemetry.addData("_shoulder ", _shoulder.getPosition());
        //telemetry.addData("_shoulder direction", _shoulder.getDirection());
        telemetry.addData("_elbow ", _elbow.getPosition());
        //telemetry.addData("_elbow direction", _elbow.getDirection());
        telemetry.addData("_grip ", _grip.getPosition());
        //telemetry.addData("_grip direction", _grip.getDirection());
        telemetry.update();
    }


    private void replayActions(ArrayList<String> list) {
        for (int i = 0; i < list.size(); i++) {
            String s = Integer.toString(i);
            String actionName = list.get(i);
            s += actionName;
            telemetry.addData("replay: ", s);

            //
            //"platform_left"
            //"platform_left@10"
            //"platform_left @ 10 @ 0.05"
            //
            String[] splitStrings = actionName.split("@", 3);
            for (int k = 0; k < splitStrings.length; k++) {
                splitStrings[k] = splitStrings[k].trim();
            }
            if (splitStrings.length == 0) {
                return;
            }
            boolean isWheelAction = splitStrings[0].startsWith("wheel");
            double localPerStepSize = perStepSize;
            int repeatTimes = 1;
            double distance = 1.0;
            double speed = 0.5;
            if (splitStrings.length >= 3) {
                localPerStepSize = Double.parseDouble(splitStrings[2]);
                if (localPerStepSize <= 0 || localPerStepSize >= 1)
                    localPerStepSize = perStepSize;
                speed = localPerStepSize;
            }
            if (splitStrings.length >= 2) {
                if (isWheelAction)
                    distance = Double.parseDouble(splitStrings[1]);
                else
                    repeatTimes = Integer.parseInt(splitStrings[1]);
            }

            if (splitStrings[0].contains("sleep")) {
                sleep(repeatTimes);
            }
            else if (splitStrings[0].startsWith("time_wheel")) {
                timeWheel(splitStrings[0], repeatTimes, speed);
            }
            else if (isWheelAction) {
                wheel(splitStrings[0], distance, speed, 10);
            }
            else {
                for (int j = 0; j < repeatTimes; j++) {
                    playAction(splitStrings[0], false);
                    sleep(100);
                }
            }
        }
        telemetry.update();
    }

    public void timeWheel(String direction, int timeMilliSeconds, double speed) {
        wheelRunTime.reset();

        //default: left
        double frontLeftPower = -speed;
        double frontRightPower = speed;
        double rearLeftPower = speed;
        double rearRightPower = -speed;

        if (direction.contains("time_wheel_right")) {
            frontLeftPower = -frontLeftPower;
            frontRightPower = -frontRightPower;
            rearLeftPower = -rearLeftPower;
            rearRightPower = -rearRightPower;
        }
        else if (direction.contains("time_wheel_forward")) {
            frontLeftPower = speed;
            frontRightPower = speed;
            rearLeftPower = speed;
            rearRightPower = speed;
        }
        else if (direction.contains("time_wheel_back")) {
            frontLeftPower = -speed;
            frontRightPower = -speed;
            rearLeftPower = -speed;
            rearRightPower = -speed;
        }

        _fl.setPower(frontLeftPower);
        _fr.setPower(frontRightPower);
        _rl.setPower(rearLeftPower);
        _rr.setPower(rearRightPower);

        while (wheelRunTime.milliseconds() < timeMilliSeconds) {
            telemetry.addData("Direction", direction);
            telemetry.addData("speed", speed);
            telemetry.addData("run for ms:", timeMilliSeconds);
            telemetry.addData("time", wheelRunTime.milliseconds());
            telemetry.update();
        }

        // Stop all motion;
        _fl.setPower(0);
        _fr.setPower(0);
        _rl.setPower(0);
        _rr.setPower(0);
    }

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 512 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265);
    private ElapsedTime     wheelRunTime = new ElapsedTime();

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void wheel(String direction, double distanceInches, double speed, double timeoutS) {
        int newFrontLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newRearLeftTarget = 0;
        int newRearRightTarget = 0;

        // reset the timeout time and start motion.
        wheelRunTime.reset();
        double frontLeftPower = speed;
        double frontRightPower = speed;
        double rearLeftPower = speed;
        double rearRightPower = speed;

        if (direction.contains("forward") || direction.equals("wheel_forward") ) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = _fl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = speed;
            frontRightPower = 0 - speed;
            rearLeftPower = speed;
            rearRightPower = 0 - speed;
        }
        else if (direction.contains("back") || direction.equals("wheel_back") ) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = _fl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = speed;
            frontRightPower = speed;
            rearLeftPower = speed;
            rearRightPower = speed;
        }
        else if (direction.equals("wheel_turn_left") ) {
            newFrontLeftTarget = _fl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = 0 - speed;
            frontRightPower = speed;
            rearLeftPower = 0 - speed;
            rearRightPower = speed;
        }
        else if (direction.equals("wheel_turn_right") ) {
            newFrontLeftTarget = _fl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = speed;
            frontRightPower = 0 - speed;
            rearLeftPower = speed;
            rearRightPower = 0 - speed;
        }
        else if (direction.equals("wheel_left") ) {
            newFrontLeftTarget = _fl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = speed;
            frontRightPower = 0 - speed;
            rearLeftPower = 0 - speed;
            rearRightPower = speed;
        }
        else if (direction.equals("wheel_right") ) {
            newFrontLeftTarget = _fl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);

            frontLeftPower = 0 - speed;
            frontRightPower = speed;
            rearLeftPower = speed;
            rearRightPower = 0 - speed;
        }

        _fl.setTargetPosition(newFrontLeftTarget);
        _fr.setTargetPosition(newFrontRightTarget);
        _rl.setTargetPosition(newRearLeftTarget);
        _rr.setTargetPosition(newRearRightTarget);

        // Turn On RUN_TO_POSITION
        _fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _fl.setPower(frontLeftPower);
        _fr.setPower(frontRightPower);
        _rl.setPower(rearLeftPower);
        _rr.setPower(rearRightPower);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (wheelRunTime.seconds() < timeoutS) &&
                (_fl.isBusy() && _fr.isBusy() && _rl.isBusy() && _rr.isBusy())) {

            // Display it for the driver.
            telemetry.addData("direction: ", direction);
            telemetry.addData("DistanceInches: ", distanceInches);
            telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFrontLeftTarget,  newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", _fl.getCurrentPosition(), _fr.getCurrentPosition(), _rl.getCurrentPosition(), _rr.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        _fl.setPower(0);
        _fr.setPower(0);
        _rl.setPower(0);
        _rr.setPower(0);

        // Turn off RUN_TO_POSITION
        // RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER
        _fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sleep(100);   // optional pause after each move.
    }
 }


