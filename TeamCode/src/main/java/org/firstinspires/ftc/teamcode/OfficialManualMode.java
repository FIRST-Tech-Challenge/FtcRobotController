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
    public ArrayList<String> presetActions1 = new ArrayList<String>(Arrays.asList(
            "wheel_forward @10 @0.5", // wheel_left, wheel_right, wheel_back 10inch and speed is 0.5
            "sleep @200", // sleep for 200ms
            "wheel_back @5 @0.5",
            "wheel_left @5 @0.5",
            "wheel_right @5 @0.5",
            "platform_left @20", //platform left 10 times the perStepSize
            "shoulder_up@20",
            "elbow_up @10"
    ));
    public ArrayList<String> presetActionsPad2X = new ArrayList<String>(Arrays.asList(
            "wheel_forward @10 @0.5"
    ));
    public ArrayList<String> presetActionsPad2Y = new ArrayList<String>(Arrays.asList(
            "wheel_back @5 @0.3"
    ));
    public ArrayList<String> presetActionsPad2A = new ArrayList<String>(Arrays.asList(
            "wheel_left @5 @0.2"
    ));
    public ArrayList<String> presetActionsPad2B = new ArrayList<String>(Arrays.asList(
            "wheel_right @5 @0.1"
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

        _platform = hardwareMap.get(Servo.class, "platform");
        _grip = hardwareMap.get(Servo.class, "grip");
        _elbow = hardwareMap.get(Servo.class, "elbow");
        _shoulder = hardwareMap.get(Servo.class, "shoulder");

        _platform.scaleRange(0.0, 1.0);
        _shoulder.scaleRange(0.0, 1.0);
        _elbow.scaleRange(0.0, 1.0);
        _grip.scaleRange(0.0, 1.0);

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
            resetToPresetPosition(0);
            replayActions(presetActions1);
            return;
        }
        if (gamepad1.b) {
            resetToPresetPosition(0);
            //replayActions(presetActions2);
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



    private void controlArm(){
            double _armPosition = _shoulder.getPosition();
        if (gamepad2.left_bumper) {
            playAction("left_bumper", true);
            //_platform.setPosition(_platform.getPosition() - 0.05);
        }
        if (gamepad2.right_bumper) {
            playAction("right_bumper", true);
            //_platform.setPosition(_platform.getPosition() + 0.05);
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
        if (gamepad2.x) {
            playAction("x", true);
            //telemetry.addData("WEEWOO IT GO UP SHOLDER", 0.7);
            //_elbow.setPosition(0.7);
            //_shoulder.setPosition(0.7);
        }

        if (gamepad2.a) {
            //telemetry.addData("I SWEAR SHOUOLDER MOVE", 0);
            playAction("a", true);
            //_shoulder.setPosition(1);
        }
        if (gamepad2.b) {
            playAction("b", true);
            //_elbow.setPosition(0);
        }
        if (gamepad2.y) {
            playAction("y", true);
            //_elbow.setPosition(1);
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
            //_elbow.setPosition(_elbow.getPosition() + 0.1); // or elbow.setPosition(elpos + 0.05);
        }
        if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) {
            playAction("right_trigger", true);
            //_elbow.setPosition(_elbow.getPosition() - 0.1);
        }

        //_elbow.setPosition(_armPosition);

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
        if (actionName.equals("left_bumper") || actionName.equals("dpad_left") || actionName.equals("platform_left")) {
            if (_platform.getPosition() >= perStepSize)
                _platform.setPosition(_platform.getPosition() - perStepSize);
        }
        else if (actionName.equals("right_bumper") || actionName.equals("dpad_right") || actionName.equals("platform_right")) {
            if (_platform.getPosition() <= (1 - perStepSize))
                _platform.setPosition(_platform.getPosition() + perStepSize);
        }
        else if (actionName.equals("dpad_up") || actionName.equals("shoulder_up")) {
            if (_shoulder.getPosition() >= perStepSize)
                _shoulder.setPosition(_shoulder.getPosition() - perStepSize);
        }
        else if (actionName.equals("dpad_down") || actionName.equals("shoulder_down")) {
            if (_shoulder.getPosition() <= (1 - perStepSize))
                _shoulder.setPosition(_shoulder.getPosition() + perStepSize);
        }
        else if (actionName.equals("x")) {
            replayActions(presetActionsPad2X);
        }
        else if (actionName.equals("a")) {
            replayActions(presetActionsPad2A);
        }
        else if (actionName.equals("b")) {
            replayActions(presetActionsPad2B);
        }
        else if (actionName.equals("y")) {
            replayActions(presetActionsPad2Y);
        }
        else if (actionName.equals("left_stick_button")) {
            if (_grip.getPosition() <= (1 - perStepSize))
                _grip.setPosition(_grip.getPosition() + perStepSize);
            //_grip.setPosition(1);
        }
        else if (actionName.equals("right_stick_button")) {
            if (_grip.getPosition() >= perStepSize)
                _grip.setPosition(_grip.getPosition() - perStepSize);
        }
        else if (actionName.equals("left_trigger") || actionName.equals("elbow_up")) {
            if (_elbow.getPosition() < (1 - perStepSize)) {
                _elbow.setPosition(_elbow.getPosition() + perStepSize);
            }
        }
        else if (actionName.equals("right_trigger") || actionName.equals("elbow_down")) {
            if (_elbow.getPosition() > perStepSize) {
                _elbow.setPosition(_elbow.getPosition() - perStepSize);
            }
        }

    }

    private void resetToPresetPosition(int presetMode) {
        telemetry.addData("Preset position", presetMode);
        if (presetMode == 0) {
            // center the control arms
            _platform.setPosition(0.35);
            _shoulder.setPosition(0.5);
            _elbow.setPosition(0.7);
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
        telemetry.addData("_shoulder ", _shoulder.getPosition());
        telemetry.addData("_elbow ", _elbow.getPosition());
        telemetry.addData("_grip ", _grip.getPosition());
        telemetry.update();
    }
//
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
                splitStrings[k].trim();
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
            if (isWheelAction) {
                moveTo(splitStrings[0], distance, speed, 10);
            }
            else {
                if (splitStrings[0].equals("sleep")) {
                    sleep(repeatTimes);
                }
                else {
                    for (int j = 0; j < repeatTimes; j++) {
                        playAction(splitStrings[0], false);
                        sleep(100);
                    }
                }
            }
        }
        telemetry.update();
    }

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private ElapsedTime     wheelRunTime = new ElapsedTime();

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void moveTo(String direction,
                             double distanceInches, double speed,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = _fl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newFrontRightTarget = _fr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearLeftTarget = _rl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
            newRearRightTarget = _rr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            wheelRunTime.reset();
            double frontLeftPower = speed;
            double frontRightPower = speed;
            double rearLeftPower = speed;
            double rearRightPower = speed;

            if (direction.equals("forward") || direction.equals("wheel_forward") ) {
                // default
            }
            else if (direction.equals("back") || direction.equals("wheel_back") ) {
                newFrontLeftTarget = _fl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
                newFrontRightTarget = _fr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
                newRearLeftTarget = _rl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
                newRearRightTarget = _rr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);

                frontLeftPower = 0 - speed;
                frontRightPower = 0 - speed;
                rearLeftPower = 0 - speed;
                rearRightPower = 0 - speed;
            }
            else if (direction.equals("left") || direction.equals("wheel_left") ) {
                newFrontLeftTarget = _fl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
                newFrontRightTarget = _fr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
                newRearLeftTarget = _rl.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
                newRearRightTarget = _rr.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);

                frontLeftPower = 0 - speed;
                frontRightPower = speed;
                rearLeftPower = speed;
                rearRightPower = 0 - speed;
            }
            else if (direction.equals("right") || direction.equals("wheel_right") ) {
                newFrontLeftTarget = _fl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
                newFrontRightTarget = _fr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
                newRearLeftTarget = _rl.getCurrentPosition() - (int)(distanceInches * COUNTS_PER_INCH);
                newRearRightTarget = _rr.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);

                frontLeftPower = speed;
                frontRightPower = 0 - speed;
                rearLeftPower = 0 - speed;
                rearRightPower = speed;
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
            _fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            _fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            _rl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            _rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //sleep(100);   // optional pause after each move.
        }
    }
 }


