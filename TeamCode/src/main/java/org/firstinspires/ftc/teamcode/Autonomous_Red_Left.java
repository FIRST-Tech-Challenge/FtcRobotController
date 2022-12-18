package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "red_left", group = "Match")
public class Autonomous_Red_Left extends LinearOpMode {
    public String workingMode = "red_left";
    public boolean autoMode = true;
    public boolean useCamera = true;
    public boolean debugMode = false;

    public int defaultParkingPosition = 3;
    public double inchesOneSquare = 26;
    public boolean useOwnAIModel = true;
    public int ownAIModelType = 1; // 1 for shape, 2 for flowers
    private DcMotor _fl, _fr, _rl, _rr;
    private Servo _grip, _platform, _elbow, _shoulder;
    private boolean logMode = false;
    private ArrayList<String> logArray = new ArrayList<>();
    private boolean firstTime = true;

    private ElapsedTime recentActionTime = new ElapsedTime();
    public double perStepSizePlatform = 0.002;
    public double perStepSizeShoulder = 0.002;
    public double perStepSizeElbow = 0.002;
    public double perStepSizeGrip = 0.01;
    public boolean enablePad1Control = false;
    public int minTimeOfTwoOperations = 20; //milliseconds, 0.05 second
    public double ratioPad2WheelSpeed = 0.1; //pad2 can control wheel at the ratio speed of pad1, 0 means pad2 stick can't wheels, 1 means same as pad1

    public double shoulderDefaultPosition = 0.536;
    public double shoulderMaxPosition = 0.708; // 0.72 vertical
    public double shoulderMiddlePosition = 0.63;
    public double shoulderPlatformSafeMin = 0.59;
    public double shoulderMinPosition = 0.531;
    public double shoulderMinTriplePosition = 0.539;
    public double elbowMaxPosition = 0.06; // manual mode need a litter higher
    public double elbowMiddlePosition = 0.27;
    public double elbowMinPositionLeft = 0.56;
    public double elbowMinPositionRight = 0.575;
    public double elbowMinPosition = elbowMinPositionRight;
    public double elbowMinTriplePosition = 0.505;
    public double elbowDefaultPosition = 0.9; //0.85;
    public double platformMinPosition = 0.05 ;
    public double platformDefaultPosition = 0.15667 ;
    public double platformMaxPosition = 0.3;
    public double platformMinPositionTriple_Right = 0.25;
    public double platformMaxPositionTriple_Right = 0.063;
    public double platformMinPositionTriple_Left = 0.05667;
    public double platformMaxPositionTriple_Left = 0.23889;
    public double gripMinPosition = 0.178;
    public double gripMaxPosition = 0.69;
    public boolean elbowSlowMotionInitial = true;

    public double elbowBothMaxShoulderBeginPosition = 0.7; //elbow start position when both_max begin

    public int    elbowTimeBothMaxShoulderBegin = 200; // elbow remain at same position when shoulder begin up
    public int    shoulderTimeFromMinToMax = 1100;
    public double wheelTurnSpeed = 2.0;
    public double zoomRatio = 1.0;
    public int parkingPosition = defaultParkingPosition;
    public boolean prepareForManual = true;
    public boolean autoE2E = false;
    public int stepsShoulderFromMinToMax = (int) ((shoulderMaxPosition - shoulderMinTriplePosition) / perStepSizeShoulder) + 1;
    public boolean platformCheckShoulderPosition = true;
    // "wheel_forward @10 @0.5", wheel_back 10inch and speed is 0.5
    // wheel_left/wheel_right/wheel_back
    // platform and shoulder elbow remain still, position / direction not changed

    // "time_wheel_left @3500 @0.2" : go straight left for 3500 milliseconds at speed 0.2
    // "time_wheel_right @5000 @0.3"
    // "time_wheel_forward @2000 @0.3" "time_wheel_back @3000 @0.3"
    // platform / shoulder / elbow position not changed with "time_wheel_" action

    // "wheel_turn_left @14 @0.3" turn left about 90 degree at speed 0.3
    // "wheel_turn_left @28 @0.2" turn about 180 degree at speed 0.2
    // platform / shoulder / elbow direction will change at same time

    // "platform_left @10" : platform turn left 10 times the perStepSize
    // "platform_right @20
    // "shoulder_up @10" "shoulder_down @20"
    // shoulder up or down 10 times the perStepSize
    // "elbow_up @10" "elbow_down @20"
    //
    // "both_min" shoulder and elbow will go down to the lowest position, ready to grab cone
    // "both_max" shoulder and elbow will go up to the highest position, ready to put cone to the pole

    // "position_shoulder @0.4", shoulder setPosition directly to 0.4
    // "position_platform @0.8", platform setPosition directly to 0.8
    // "position_elbow @ 0.5", elbow setPosition directly to 0.5

    // "grip_max" "grip_min" "grip_open @10" "grip_close @10"
    //

    // "ai_park" get park destination using AI
    // "zoom @1.5" zoom_in ratio is 1.5
    public ArrayList<String> presetActionsLeft = new ArrayList<String>(Arrays.asList(
            "sleep @1"
    ));

    //shoulder and elbow max, not blocking camera
    //go forward some distance, and zoom in the camera
    public ArrayList<String> presetActionsStep1 = new ArrayList<String>(Arrays.asList(
            "grip_min",
            "sleep @1000",
            "both_default",
            "sleep @1000",
            "shoulder_up @10 @0.2",
            "both_max",
            "platform_default",
            "wheel_forward @16 @0.2",
            "ai_get_parkposition",
            "wheel_forward @23 @0.2",
            "sleep @100",
            "wheel_back @13 @0.2",
            "nextstep @presetActionsStep2"
    ));

    public ArrayList<String> presetActionsStep2_left = new ArrayList<String>(Arrays.asList(
            "wheel_right @31 @0.2",
            "wheel_forward @3 @0.2",
            "platform_right @25 @0.2",
            //"wheel_turn_right @8.6 @0.1",
            //"wheel_forward @1 @0.1",
            "sleep @500",
            "grip_max",
            "sleep @500",
            //"wheel_back @1 @0.1",
            //"wheel_turn_left @8.6 @0.1",
            "platform_left @25 @0.2",
            "park_ai_position",
            "wheel_back @3 @0.2",
            "sleep @100",
            //"wheel_turn_right @16.5 @0.2",
            "both_default",
            "wheel_back @1 @0.2"
    ));

    public ArrayList<String> presetActionsStep2_right = new ArrayList<String>(Arrays.asList(
            "wheel_left @31.5 @0.2",
            "wheel_forward @3 @0.2",
            "platform_left @25 @0.2",
            //"wheel_turn_left @7 @0.1",
            //"wheel_forward @1 @0.1",
            "sleep @500",
            "grip_max",
            "sleep @500",
            //"wheel_back @1 @0.1",
            //"wheel_turn_right @7 @0.1",
            "platform_right @25 @0.2",
            "park_ai_position",
            "wheel_back @2 @0.2",
            "sleep @100",
            //"wheel_turn_left @16.5 @0.2",
            "both_default",
            "wheel_back @1 @0.2"
    ));

    public ArrayList<String> presetActionsAutoE2E = new ArrayList<String>(Arrays.asList(
            "triple_min",
            "sleep @500",
            "grip_min",
            "sleep @800",
            "triple_max",
            "sleep @1500",
            "grip_max",
            "sleep @200"
    ));

    public ArrayList<String> presetActionsShoulderUpMax = new ArrayList<String>(Arrays.asList(
            "shoulder_up @85 @0.2"
    ));

    public ArrayList<String> presetActionsPad1X = new ArrayList<String>(Arrays.asList(
            "wheel_left @2 @0.3"
    ));
    public ArrayList<String> presetActionsPad1Y = new ArrayList<String>(Arrays.asList(
            "wheel_forward @2 @0.2"
    ));

    public ArrayList<String> presetActionsPad1A = new ArrayList<String>(Arrays.asList(
            "wheel_back @2 @0.2"
    ));

    public ArrayList<String> presetActionsPad1B = new ArrayList<String>(Arrays.asList(
            "wheel_right @2 @0.3"
    ));

    public ArrayList<String> presetActionsWheelTurnLeft = new ArrayList<String>(Arrays.asList(
            "wheel_turn_left @1 @0.2"
    ));

    public ArrayList<String> presetActionsWheelTurnRight = new ArrayList<String>(Arrays.asList(
            "wheel_turn_right @1 @0.2"
    ));

    public ArrayList<String> presetActionsPad2X = new ArrayList<String>(Arrays.asList(
            //"park_ai_position"
    ));
    public ArrayList<String> presetActionsPad2Y = new ArrayList<String>(Arrays.asList(
            "triple_max"
    ));
    public ArrayList<String> presetActionsPad2A = new ArrayList<String>(Arrays.asList(
            "triple_min"
    ));
    public ArrayList<String> presetActionsPad2B = new ArrayList<String>(Arrays.asList(
            "triple_middle"
    ));
    public ArrayList<String> presetActionsPad2X_2 = new ArrayList<String>(Arrays.asList(
            "platform_default"
    ));
    public ArrayList<String> presetActionsPad2Y_2 = new ArrayList<String>(Arrays.asList(
            "both_max"
    ));
    public ArrayList<String> presetActionsPad2A_2 = new ArrayList<String>(Arrays.asList(
            "both_min"
    ));
    public ArrayList<String> presetActionsPad2B_2 = new ArrayList<String>(Arrays.asList(
            "both_middle"
    ));
    public ArrayList<String> presetActionsDefault = new ArrayList<String>(Arrays.asList(
            "sleep @100"
    ));
    private ElapsedTime moreTimeToStart = new ElapsedTime();
    private boolean stopPresetAction = false;

    private String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private String TFOD_MODEL_FILE_NUMBER  = "/sdcard/FIRST/tflitemodels/shape.tflite";
    private String TFOD_MODEL_FILE_FLOWER  = "/sdcard/FIRST/tflitemodels/flower.tflite";

    private static final String[] LABELS_NUMBER = {
            "1",
            "2",
            "3"
    };

    private static final String[] LABELS_FLOWER = {
            "dandelion (0)",
            "daisy (1)",
            "sunflowers (3)",
            "roses (4)",
            "abc (5)"
    };

    private static final String[] LABELS_BUILTIN = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "ATLKdnj/////AAABmVUesrWAHkpqtkg5toiScOqGWcxBnaVXzQ6AYKnG4ytMQUy2LoJpp3DUYKoiA2EZluhn1YMc92J8CBE+yPxU8WrJoIYsxjuZT6J/FNft57D7HkvDvOcMVGNy3TdGWv2oLCIdDFYC20nAL1OlD2dblASXzpyWaKsI1tPJtBisQnMRyDa4ytwk5U1jhlGUsVAFg0xdyMwOsNKQALO/FUea4shIGLihl2RQtRKawpB0bou99vaxtAqGcIH06ItbIKIegF/z3bpO/a7GKECeAInSJ3UJvQmey0aHeLu/KXKkmw9bRdgYCyNbtgdSd+cHr+ZdmNp0/yV3xqeWr8DQsimFkv5gDMwGfUXlEua6sb85kHwL";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {
        if (workingMode.contains("left")) {
            elbowMinPosition = elbowMinPositionLeft;
        }
   
        // Declare our motors
        // Make sure your ID's match your configuration
        _fl = hardwareMap.dcMotor.get("frontLeft");
        _fr = hardwareMap.dcMotor.get("frontRight");
        _rl = hardwareMap.dcMotor.get("rearLeft");
        _rr = hardwareMap.dcMotor.get("rearRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        _fl.setDirection(DcMotorSimple.Direction.FORWARD);
        _fr.setDirection(DcMotorSimple.Direction.REVERSE);
        _rl.setDirection(DcMotorSimple.Direction.FORWARD);
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

        if (useCamera) {
            initVuforia();
            initTfod();
            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(zoomRatio, 16.0/9.0);
            }
        }

        //_shoulder.resetDeviceConfigurationForOpMode();
        //_platform.resetDeviceConfigurationForOpMode();
        //_elbow.resetDeviceConfigurationForOpMode();
        //_grip.resetDeviceConfigurationForOpMode();
        //_platform.scaleRange(0.0, 1.0);
        //_shoulder.scaleRange(0.0, 1.0);
        //_elbow.scaleRange(0.0, 1.0);
        //_grip.scaleRange(0.0, 1.0);

        telemetry.addData("_grip device name", _grip.getDeviceName());
        telemetry.addData("_grip manufacturer", _grip.getManufacturer());
        telemetry.update();

        if (autoMode) {
            //_grip.setPosition(gripMinPosition);
        }

       // _elbow.scaleRange(0.2,0.8);
        waitForStart();

        if (isStopRequested()) return;

        Thread threadWheel = new Thread(new MultithreadingWheel());
        Thread threadArm = new Thread(new MultithreadingArm());

        moreTimeToStart.reset();
        while (opModeIsActive()) {
                if (moreTimeToStart.milliseconds() < 100) {
                telemetry.addData("Waiting millisecond: ", moreTimeToStart.milliseconds()  );
                telemetry.update();
                continue;
                // do nothing;
            }
            if (firstTime) {
                firstTime = false;
                if (autoMode) {
                    //_grip.setPosition(gripMinPosition);
                    //resetToPresetPosition(0);
                }
                else {
                    _shoulder.setPosition(shoulderPlatformSafeMin);
                    _elbow.setPosition(elbowDefaultPosition);
                }
                if (autoMode == false) {
                    threadWheel.start();
                    threadArm.start();
                }
                if (useCamera) {
                    //ConceptTensorFlowObjectDetectionWebcam camera = new ConceptTensorFlowObjectDetectionWebcam();
                    //camera.setCameraName(cameraName);
                }
                if (autoMode == true) {
                    //waitElapsedTime(200);
                    replayActions(presetActionsStep1);
                }
            }
            if (autoMode == true) {
                    //controlArm();
                    //controlWheels();
            } else {
                if (gamepad2.left_stick_button && gamepad2.options) {
                    autoE2E = true;
                    int maxRound = 15;
                    while (autoE2E && maxRound > 0) {
                        replayActions(presetActionsAutoE2E);
                        maxRound --;
                    }
                }
            }
            //sleep(1);
        }
        if (autoMode == false) {
            threadWheel.interrupt();
            threadArm.interrupt();
        }
        if (autoMode && useCamera) {
            if (tfod != null) {
                tfod.deactivate();
                tfod.shutdown();
                tfod = null;
            }
        }
    }

    private ElapsedTime resetServoPositionTimer = new ElapsedTime();
    private void resetToPresetPosition(int presetMode) {
        //telemetry.addData("Preset position", presetMode);
        if (presetMode == 0) {
            logAction("Initial");
            _shoulder.setPosition(shoulderDefaultPosition);
            if (elbowSlowMotionInitial) {
                waitElapsedTime(600);
                _elbow.setPosition(elbowDefaultPosition);
                int remainTime = 1000;
                int perStepSleepTime = 50;
                double elbowPosition = _elbow.getPosition();
                double stepSize = (elbowPosition - elbowBothMaxShoulderBeginPosition) / (remainTime / perStepSleepTime);
                while (remainTime > 0) {
                    telemetry.addData("stepSize", stepSize);
                    telemetry.addData("remainTime", remainTime);
                    logAction("both_max");
                    elbowPosition -= stepSize;
                    if (elbowPosition >= elbowBothMaxShoulderBeginPosition)
                        _elbow.setPosition(elbowPosition);
                    waitElapsedTime(perStepSleepTime);
                    remainTime -= perStepSleepTime;
                }
            }
            else {
                _elbow.setPosition(elbowDefaultPosition);
            }
            _platform.setPosition(platformDefaultPosition);
            //replayActions(presetActionsShoulderUp);
        }
    }

    private boolean pwmEnable = true;
    public void resetServoPosition() {
        if (resetServoPositionTimer.seconds() > 5) {
            //_platform.getController().resetDeviceConfigurationForOpMode();
            _platform.getController().setServoPosition(0, 0.5);
            //ServoController.PwmStatus pwmStatus = _platform.getController().getPwmStatus();
            if (pwmEnable) {
                _platform.getController().pwmDisable();
                pwmEnable = false;
            }
            else {
                _platform.getController().pwmEnable();
                pwmEnable = true;
                _platform.setPosition(0.5);
            }
            //_platform.resetDeviceConfigurationForOpMode();
            //_shoulder.getController().setServoPosition(1, shoulderDefaultPosition);
            //_elbow.getController().setServoPosition(2, elbowDefaultPosition);
            resetServoPositionTimer.reset();
        }
    };

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
            replayActions(presetActionsPad1X);
            return;
        }
        if (gamepad1.y) {
            replayActions(presetActionsPad1Y);
            return;
        }
        if (gamepad1.left_trigger > 0 && gamepad1.left_bumper) {
            replayActions(presetActionsWheelTurnLeft);
            return;
        }
        if (gamepad1.right_trigger > 0 && gamepad1.right_bumper) {
            replayActions(presetActionsWheelTurnLeft);
            return;
        }
        if (debugMode && gamepad1.options) {
            //setLogMode(!logMode);
            return;
        }
        if (debugMode && gamepad1.left_stick_button && gamepad1.right_stick_button) {
            //resetServoPosition();
            return;
        }

        // enable for both controller
        double y, x , rx, denominator;
        double frontLeftPower, backLeftPower, frontRightPower, backRightPower, speedmultiplier;
        y = gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
        x = -gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
        rx = -gamepad1.right_stick_x * 0.5;

        //if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0) {
        y = gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
        x = -gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
        rx = -gamepad1.right_stick_x * 0.5;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;
        speedmultiplier = 1;
        if (gamepad1.left_trigger > 0) {
            speedmultiplier = 0.2;
        } else if (gamepad1.right_trigger > 0) {
            speedmultiplier = 2;
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

    void setElbowPositionAlongWithShoulder(double shoulderPosition) {
        double ratio = (elbowMaxPosition - elbowMinPosition) / (shoulderMaxPosition - shoulderMinPosition);
        double elbowPosition = ratio * (shoulderPosition - shoulderMinPosition) + elbowMinPosition;
        _elbow.setPosition(elbowPosition);
    }


    private void controlArm() {
        if (gamepad2.x) {
            if (gamepad2.left_stick_button) {
               replayActions(presetActionsPad2X);
            }
            else if (gamepad2.right_stick_button) {
                replayActions(presetActionsPad2X_2);
            }
            else {
                autoE2E = false;
            }
            return;
        }
        if (gamepad2.a) {
            if (gamepad2.left_stick_button) {
                replayActions(presetActionsPad2A);
            }
            else if (gamepad2.right_stick_button) {
                replayActions(presetActionsPad2A_2);
            }
            else {
                autoE2E = false;
            }
            return;
        }
        if (gamepad2.b) {
            if (gamepad2.left_stick_button) {
               replayActions(presetActionsPad2B);
            }
            else if (gamepad2.right_stick_button) {
                replayActions(presetActionsPad2B_2);
            }
            else {
                autoE2E = false;
            }
            return;
        }
        if (gamepad2.y) {
            if (gamepad2.left_stick_button) {
               replayActions(presetActionsPad2Y);
            }
            else if (gamepad2.right_stick_button) {
                replayActions(presetActionsPad2Y_2);
            }
            else {
                autoE2E = false;
            }
            return;
        }

        //gamepad1 operation to be removed later, test only
        if (debugMode && enablePad1Control && gamepad1.dpad_up) {
            playAction("shoulder_up_only", true);
            return;
        }
        if (debugMode && enablePad1Control && gamepad1.dpad_down) {
            playAction("shoulder_down_only", true);
            return;
        }
        if (debugMode && enablePad1Control && gamepad1.left_trigger > 0) {
            playAction("grip_open", true);
            return;
        }
        if (debugMode && enablePad1Control && gamepad1.right_trigger > 0) {
            playAction("grip_close", true);
            return;
        }
        if (gamepad1.options) {
            logAction("current status");
            return;
        }
        if (gamepad2.left_bumper) {
            playAction("grip_max", true);
            return;
        }
        if (gamepad2.right_bumper) {
            playAction("grip_min", true);
            return;
        }
        if (gamepad2.dpad_up) {
            playAction("shoulder_up", true);
            return;
        }
        if (gamepad2.dpad_down) {
            playAction("shoulder_down", true);
            return;
        }
        if (gamepad2.dpad_left || (enablePad1Control && gamepad1.dpad_left)) {
            playAction("platform_left", true);
            return;
        }
        if (gamepad2.dpad_right || (enablePad1Control && gamepad1.dpad_right)) {
            playAction("platform_right", true);
            return;
        }
        if ((gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) ||
                (enablePad1Control && gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)) {
            playAction("elbow_up", true);
            return;
        }
        if ((gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0) ||
                (enablePad1Control && gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0)) {
            playAction("elbow_down", true);
            return;
        }
        if (gamepad2.left_stick_button) {
            //playAction("left_stick_button", true);
            return;
        }
        if (gamepad2.right_stick_button) {
            //playAction("right_stick_button", true);
            return;
        }

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
        if (actionName.equals("platform_left")) {
            if (platformCheckShoulderPosition && _shoulder.getPosition() < shoulderPlatformSafeMin)
                return;
            double targetPosition = _platform.getPosition() + perStepSizePlatform;
            if (targetPosition > platformMaxPosition) {
                targetPosition = platformMaxPosition;
            }
            _platform.setPosition(targetPosition);
        }
        else if (actionName.equals("platform_right")) {
            if (platformCheckShoulderPosition && _shoulder.getPosition() < shoulderPlatformSafeMin)
                return;
            double targetPosition = _platform.getPosition() - perStepSizePlatform;
            if (targetPosition < platformMinPosition) {
                targetPosition = platformMinPosition;
            }
            _platform.setPosition(targetPosition);
        }
        else if (actionName.equals("platform_default")) {
            if (platformCheckShoulderPosition && _shoulder.getPosition() < shoulderPlatformSafeMin)
                return;
            _platform.setPosition(platformDefaultPosition);
        }
        else if (actionName.equals("shoulder_up_only")) {
            double currentShoulderPosition = _shoulder.getPosition();
            if (currentShoulderPosition + perStepSizeShoulder <= shoulderMaxPosition) {
                _shoulder.setPosition(currentShoulderPosition + perStepSizeShoulder);
            }
            else {
                _shoulder.setPosition(shoulderMaxPosition);
            }
        }
        else if (actionName.equals("shoulder_down_only")) {
            double currentShoulderPosition = _shoulder.getPosition();
            if (currentShoulderPosition - perStepSizeShoulder >= shoulderMinPosition) {
                _shoulder.setPosition(currentShoulderPosition - perStepSizeShoulder);
            }
            else {
                _shoulder.setPosition(shoulderMinPosition);
            }
        }
        else if (actionName.equals("shoulder_up")) {
            double currentShoulderPosition = _shoulder.getPosition();
            if (currentShoulderPosition + perStepSizeShoulder <= shoulderMaxPosition) {
                _shoulder.setPosition(currentShoulderPosition + perStepSizeShoulder);
                setElbowPositionAlongWithShoulder(currentShoulderPosition + perStepSizeShoulder);
            }
            else {
                _shoulder.setPosition(shoulderMaxPosition);
                setElbowPositionAlongWithShoulder(shoulderMaxPosition);
            }
        }
        else if (actionName.equals("shoulder_down")) {
            double currentShoulderPosition = _shoulder.getPosition();
            if (currentShoulderPosition - perStepSizeShoulder >= shoulderMinPosition) {
                _shoulder.setPosition(currentShoulderPosition - perStepSizeShoulder);
                setElbowPositionAlongWithShoulder(currentShoulderPosition - perStepSizeShoulder);
            }
            else {
                _shoulder.setPosition(shoulderMinPosition);
                setElbowPositionAlongWithShoulder(shoulderMinPosition);
            }
        }
        else if (actionName.equals("grip_open")) {
            if (_grip.getPosition() < gripMaxPosition)
                _grip.setPosition(_grip.getPosition() + perStepSizeGrip);
        }
        else if (actionName.equals("grip_max")) {
            _grip.setPosition(gripMaxPosition);
        }
        else if (actionName.equals("grip_close")) {
            if (_grip.getPosition() > gripMinPosition)
                _grip.setPosition(_grip.getPosition() - perStepSizeGrip);
        }
        else if (actionName.equals("grip_min")) {
            _grip.setPosition(gripMinPosition);
        }
        else if (actionName.equals("elbow_up")) {
            if (_elbow.getPosition() > perStepSizeElbow) {
                _elbow.setPosition(_elbow.getPosition() - perStepSizeElbow);
            }
        }
        else if (actionName.equals("elbow_down")) {
            if (_elbow.getPosition() < (1 - perStepSizeElbow)) {
                _elbow.setPosition(_elbow.getPosition() + perStepSizeElbow);
            }

        }

    }


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
        telemetry.addData("_platform servo ", _platform.getController().getServoPosition(0));
        telemetry.addData("_shoulder ", _shoulder.getPosition());
        telemetry.addData("_shoulder servo ", _shoulder.getController().getServoPosition(1));
        telemetry.addData("_elbow ", _elbow.getPosition());
        telemetry.addData("_elbow servo ", _elbow.getController().getServoPosition(2));
        telemetry.addData("_grip ", _grip.getPosition());
        telemetry.addData("_grip servo ", _grip.getController().getServoPosition(3));
        telemetry.addData("parkingPosition", parkingPosition);
        telemetry.update();
    }


    private void replayActions(ArrayList<String> list) {
        for (int i = 0; i < list.size(); i++) {
            if (gamepad2.right_stick_button) {
                stopPresetAction = false;
                return;
            }
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
            int repeatTimes = 1;

            if (splitStrings[0].contains("sleep")) {
                repeatTimes = Integer.parseInt(splitStrings[1]);
                waitElapsedTime(repeatTimes);
            }
            else if (splitStrings[0].contains("ai_get_parkposition")) {
                aiGetParkPosition();
            }
            else if (splitStrings[0].startsWith("park_ai_position")) {
                parkAIPosition(splitStrings);
            }
            else if (splitStrings[0].contains("zoom")) {
                zoomRatio = Double.parseDouble(splitStrings[1]);
            }
            else if (splitStrings[0].contains("nextstep")) {
                nextStep(splitStrings);
            }
            else if (splitStrings[0].startsWith("both")) {
                shoulderElbowBoth(splitStrings, 2, 500);
            }
            else if (splitStrings[0].startsWith("triple")) {
                platformShoulderElbowTriple(splitStrings);
            }
            else if (splitStrings[0].startsWith("time_wheel")) {
                timeWheel(splitStrings[0], splitStrings[1], splitStrings[2]);
            }
            else if (splitStrings[0].startsWith("position") && splitStrings.length >= 2) {
                position(splitStrings[0], splitStrings[1]);
            }
            else if (isWheelAction && splitStrings.length >= 3) {
                wheel(splitStrings[0], splitStrings[1], splitStrings[2], 60);
            }
            else {
                if (splitStrings.length >= 2)
                    repeatTimes = Integer.parseInt(splitStrings[1]);
                for (int j = 0; j < repeatTimes; j++) {
                    if (gamepad2.right_stick_button)
                    {
                        stopPresetAction = false;
                        return;
                    }
                    playAction(splitStrings[0], false);
                    waitElapsedTime(10);
                }
            }
        }
        telemetry.update();

    }

    ArrayList<String> getArrayListByName(String name) {
        if (name.equals("presetActionsStep2_left")) {
            return presetActionsStep2_left;
        }
        else if (name.equals("presetActionsStep2_right")) {
            return presetActionsStep2_right;
        }
        else {
            return presetActionsDefault;
        }
    }

    void nextStep(String[] splitStrings) {
        String direction = "left";
        if (workingMode.equals("red_left") || workingMode.equals("blue_left")) {
            direction = "_left";
        }
        else {
            direction = "_right";
        }
        String actions = splitStrings[1] + direction;
        ArrayList<String> arrlistActions = getArrayListByName(actions);

        replayActions(arrlistActions);
    }


    public void shoulderElbowBoth(String[] splitStrings, int type, int waitShoulderMaxTime) {
        if (splitStrings[0].equals("both_min")) {
            double shoulderPosition = _shoulder.getPosition();
            if (type == 3) {
                _shoulder.setPosition(shoulderMinTriplePosition);
            }
            else {
            _shoulder.setPosition(shoulderMinPosition);
            }
            if (shoulderPosition < shoulderMaxPosition - (shoulderMaxPosition - shoulderMinPosition) * 2 / 3) {
                _elbow.setPosition(type == 3 ? elbowMinTriplePosition : elbowMinPosition);
            }
            else {
                int remainTime = shoulderTimeFromMinToMax;
                int perStepSleepTime = 20;
                double elbowPosition = _elbow.getPosition();
                double elbowTargetPosition = type == 3 ? elbowMinTriplePosition : elbowMinPosition;
                double stepSize = (elbowTargetPosition - elbowPosition) / (remainTime / perStepSleepTime);
                while (remainTime > 0) {
                    telemetry.addData("stepSize", stepSize);
                    telemetry.addData("remainTime", remainTime);
                    logAction("both_min");
                    elbowPosition += stepSize;
                    _elbow.setPosition(elbowPosition < elbowTargetPosition ? elbowPosition : elbowTargetPosition);
                    waitElapsedTime(perStepSleepTime);
                    remainTime -= perStepSleepTime;
                }
            }
            //_grip.setPosition(gripMaxPosition);
        }
        else if (splitStrings[0].equals("both_max")) {
            double shoulderPosition = _shoulder.getPosition();
            _shoulder.setPosition(shoulderMaxPosition);
            _elbow.setPosition(elbowBothMaxShoulderBeginPosition);
            //waitElapsedTime(elbowTimeBothMaxShoulderBegin);
            if (shoulderPosition > (shoulderMinPosition + (shoulderMaxPosition - shoulderMinPosition) * 2 / 3)) {
                _elbow.setPosition(elbowMaxPosition);
            }
            else {
                waitElapsedTime(waitShoulderMaxTime);
                _elbow.setPosition(elbowMaxPosition);
                /*
                int remainTime = shoulderTimeFromMinToMax - elbowTimeBothMaxShoulderBegin;
                int perStepSleepTime = 20;
                double stepSize = (elbowBothMaxShoulderBeginPosition - elbowMaxPosition) / (remainTime / perStepSleepTime);
                double elbowPosition = elbowBothMaxShoulderBeginPosition;
                while (remainTime > 0) {
                    telemetry.addData("stepSize", stepSize);
                    telemetry.addData("remainTime", remainTime);
                    logAction("both_max");
                    elbowPosition -= stepSize;
                    if (elbowPosition >= elbowMaxPosition)
                        _elbow.setPosition(elbowPosition);
                    waitElapsedTime(perStepSleepTime);
                    remainTime -= perStepSleepTime;
                }
                */
            }
            //logAction("both_max_end");
            //_elbow.setPosition(0.5);
            //sleep(2000);
        }
        else if (splitStrings[0].equals("both_middle")) {
            double shoulderPosition = _shoulder.getPosition();
            _shoulder.setPosition(shoulderMiddlePosition);
            waitElapsedTime(200);
            _elbow.setPosition(elbowMiddlePosition);
        }
        else if (splitStrings[0].equals("both_default")) {
            _shoulder.setPosition(shoulderDefaultPosition);
            _elbow.setPosition(elbowDefaultPosition);
        }
    }

    public void platformShoulderElbowTriple(String[] splitStrings) {
        if (splitStrings[0].equals("triple_min")) {
            if (_shoulder.getPosition() >= shoulderMaxPosition) {
            }
            else {
                _shoulder.setPosition(shoulderMaxPosition);
                waitElapsedTime(300);
            }
            if (workingMode.contains("right")) {
                _platform.setPosition(platformMinPositionTriple_Right);
            }
            else {
                _platform.setPosition(platformMinPositionTriple_Left);
            }
            waitElapsedTime(100);
            String[] actionString = {"both_min"};
            shoulderElbowBoth(actionString, 3, 500);
            //_grip.setPosition(gripMaxPosition);
        }
        else if (splitStrings[0].equals("triple_max")) {
            /*
            if (_shoulder.getPosition() >= shoulderPlatformSafeMin) {
            }
            else {
                _shoulder.setPosition(shoulderPlatformSafeMin);
                waitElapsedTime(300);
            }
            */
            /*
            _shoulder.setPosition(shoulderMaxPosition);
            _elbow.setPosition(elbowBothMaxShoulderBeginPosition);
            waitElapsedTime(elbowTimeBothMaxShoulderBegin);
            _elbow.setPosition(elbowMaxPosition);
            int remainTime = shoulderTimeFromMinToMax - elbowTimeBothMaxShoulderBegin;
            int perStepSleepTime = 20;
            double stepSize = (elbowBothMaxShoulderBeginPosition - elbowMaxPosition) / (remainTime / perStepSleepTime);
            double elbowPosition = elbowBothMaxShoulderBeginPosition;
            while (remainTime > 0) {
                telemetry.addData("stepSize", stepSize);
                telemetry.addData("remainTime", remainTime);
                logAction("both_max");
                elbowPosition -= stepSize;
                if (elbowPosition >= elbowMaxPosition)
                    _elbow.setPosition(elbowPosition);
                waitElapsedTime(perStepSleepTime);
                remainTime -= perStepSleepTime;
            }
            */
            replayActions(presetActionsShoulderUpMax);
            if (workingMode.contains("right")) {
                _platform.setPosition(platformMaxPositionTriple_Right);
            }
            else {
                _platform.setPosition(platformMaxPositionTriple_Left);
        }
            //logAction("both_max_end");
            //_elbow.setPosition(0.5);
            //sleep(2000);
        }
        else if (splitStrings[0].equals("triple_middle")) {
            double shoulderPosition = _shoulder.getPosition();
            _shoulder.setPosition(shoulderMiddlePosition);
            waitElapsedTime(200);
            _elbow.setPosition(elbowMiddlePosition);
        }
        else if (splitStrings[0].equals("triple_default")) {
            _shoulder.setPosition(shoulderDefaultPosition);
            _elbow.setPosition(elbowDefaultPosition);
        }
    }


    public void position(String target, String sPosition) {
        double targetPosition = Double.parseDouble(sPosition);
        if (targetPosition < 0 || targetPosition > 1)
            return;
        if (target.equals("position_platform")) {
            _platform.setPosition(targetPosition);
        }
        else if (target.equals("position_shoulder")) {
            _shoulder.setPosition(targetPosition);
        }
        else if (target.equals("position_elbow")) {
            _elbow.setPosition(targetPosition);
        }
    }

    public void timeWheel(String direction, String sMilliSeconds, String sSpeed) {
        int timeMilliSeconds = Integer.parseInt(sMilliSeconds);
        double speed = Double.parseDouble(sSpeed);
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
    static final double     COUNTS_PER_MOTOR_REV    = 512 ;
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

    public void wheel(String direction, String sDistanceInches, String sSpeed, double timeoutS) {
        double distanceInches = Double.parseDouble(sDistanceInches);
        double speed = Double.parseDouble(sSpeed);
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
            telemetry.addData("parking position: ", parkingPosition);
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

    class MultithreadingWheel implements Runnable {
        public void run()
        {
            try {
                while (true) {
                    controlWheels();
                    //sleep(1);
                }
            }
            catch (Exception e) {
                System.out.println("Exception is caught");
                return;
            }
        }
    }
    class MultithreadingArm implements Runnable {
        public void run()
        {
            try {
                while (true) {
                    controlArm();
                    //sleep(1);
                }
            }
            catch (Exception e) {
                System.out.println("Exception is caught");
                return;
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        if (useOwnAIModel && ownAIModelType == 1) {
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 300;
        }
        else if (useOwnAIModel && ownAIModelType == 2) {
            tfodParameters.isModelTensorFlow2 = false;
            tfodParameters.inputSize = 300;
        }
        else {
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 300;
        }
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        if (useOwnAIModel) {
            if (ownAIModelType == 1) {
                tfod.loadModelFromFile(TFOD_MODEL_FILE_NUMBER, LABELS_NUMBER);
            }
            else {
                tfod.loadModelFromFile(TFOD_MODEL_FILE_FLOWER, LABELS_FLOWER);
            }
        }
        else {
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS_BUILTIN);
        }
    }

    boolean aiGetParkPosition() {
        parkingPosition = defaultParkingPosition;
        if (useCamera == false) {
            return false;
        }
        ElapsedTime timeWaitAi = new ElapsedTime();;

        int type = -1;
            while (timeWaitAi.milliseconds() < 1500) {
            if (useCamera && tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        if (recognition.getConfidence() > 0.6) {
                            String s = recognition.getLabel();
                            if (useOwnAIModel) {
                                if (s.equals("1")) {
                                    type = 1;
                                } else if (s.equals("2")) {
                                    type = 2;
                                } else if (s.equals("3")) {
                                    type = 3;
                                }
                            }
                            else {
                                if (s.equals("1 Bolt")) {
                                    type = 1;
                                } else if (s.equals("2 Bulb")) {
                                    type = 2;
                                } else if (s.equals("3 Panel")) {
                                    type = 3;
                                }
                            }
                        }
                        telemetry.addData("park position", type);
                    }
                    if (timeWaitAi.milliseconds() < 500) {
                        telemetry.addData("ignore time", type);
                    }
                    else {
                        telemetry.addData("not ignore", type);
                    }
                    telemetry.update();
                }
            }
        }
        if (type == -1)
            return false;
        else {
            parkingPosition = type;
            return true;
        }
    }

    void parkAIPosition(String[] splitStrings) {
        String operation = "wheel_left";
        double distance = 5;
        // begin in red left column 3
        if (workingMode.equals("red_left")) {
            operation = "wheel_left";
            if (parkingPosition == 3) {
                distance = 3;
            }
            else if (parkingPosition == 2) {
                distance += 1 * inchesOneSquare;
            }
            else if (parkingPosition == 1) {
                distance += 2 * inchesOneSquare + 2;
            }
        }
        // begin in red right column 1
        else if (workingMode.equals("red_right")) {
            operation = "wheel_right";
            if (parkingPosition == 3) {
                distance += 2 * inchesOneSquare + 2;
            }
            else if (parkingPosition == 2) {
                distance += 1 * inchesOneSquare;
            }
            else if (parkingPosition == 1) {
                distance = 3;
            }
        }
        // begin in blue left column 1
        else if (workingMode.equals("blue_left")) {
            operation = "wheel_left";
            if (parkingPosition == 3) {
                distance += 2 * inchesOneSquare;
            }
            else if (parkingPosition == 2) {
                distance += 1 * inchesOneSquare;
            }
            else if (parkingPosition == 1) {
                distance = 0;
            }
        }
        // begin in blue right column 3
        else if (workingMode.equals("blue_right")) {
            operation = "wheel_right";
            if (parkingPosition == 3) {
                distance = 0;
            }
            else if (parkingPosition == 2) {
                distance += 1 * inchesOneSquare;
            }
            else if (parkingPosition == 1) {
                distance += 2 * inchesOneSquare;
            }
        }
        String sDistance = Double.toString(distance);
        wheel(operation, sDistance, "0.5", 10000);
    }

    void waitElapsedTime(int waitTime)
    {
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < waitTime) {
            telemetry.addData("Waiting millisecond: ", moreTimeToStart.milliseconds()  );
            telemetry.update();
            continue;
        }
    }
 }


