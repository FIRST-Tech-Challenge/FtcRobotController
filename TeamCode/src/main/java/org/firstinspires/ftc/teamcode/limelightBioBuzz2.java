package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.Arrays;
//import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
//test
import com.qualcomm.robotcore.hardware.IMU;
import java.util.List;

//test
@TeleOp(name="limelight BioBuzz2")
public class limelightBioBuzz2 extends LinearOpMode{
    private Limelight3A limelight;
    public static boolean isAuton = false;

    public enum MovePattern {
        moveStart,
        obeliskView,
        waitObeliskView,
        move1,
        alignMove1,
        waitmove1,
        move2,
        waitmove2,
        move3,
        alignMove3,
        waitmove3,
        move4,
        waitmove4,
        move5,
        waitmove5,
        move6,
        done
    };
    MovePattern movePattern = MovePattern.done;

    public enum ArtifactDetector {
        start,
        noartifact,
        purple,
        green
    };
    ArtifactDetector artifactDetector = ArtifactDetector.start;

    public enum IntakeState {
        ready,
        empty,
        hasOne,
        hasTwo,
        hasThree,
        miniStacking,
        miniStacked,
        stacked,
        waitForSlackChange,
        calculateExitDistance,
        A,
        B,
        C,
        D,
        E,
        // X,
        setTarget,
        waitForCatapult,
        intakeToOuttake,
        presentNextArtifact,
        waitForLaunch,
        done
    };

    IntakeState intakeState = IntakeState.ready;

    public enum WaitState {
        wait,
        done
    };
    WaitState waitState = WaitState.done;


    public enum ProfileState {
        accel,
        slew1,
        slew2,
        decel,
        done
    };
    ProfileState profileState = ProfileState.done;

    public enum CurveState {
        calculatingCoefficients,
        calculatingCommands,
        done
    };
    CurveState curveState = CurveState.done;

    public enum CatapultLaunchState {
        ready,
        initArmBack,
        initSlackValue,
        waitForArtifactReady,
        servoRelease,
        waitForCatapultReady,
        //servoReturn,
        armBack,
        goToSlack,
        done
    }
    CatapultLaunchState catapultLaunchState = CatapultLaunchState.done;
    // public enum MotorLed {
    //     ready,
    //     isRunning,
    //     notRunning,
    //     done
    // }
    // MotorLed motorLed = MotorLed.ready;

    // public enum ShotsLed {
    //     ready,
    //     oneShot,
    //     twoShot,
    //     threeShot,
    //     done
    // }
    // ShotsLed shotsLed = ShotsLed.ready;

    // public enum BlinkingState {
    //     on,
    //     off,
    //     done
    // }
    // BlinkingState blinkingState = BlinkingState.done;

    // CatapultLaunchState catapultLaunchState = CatapultLaunchState.done;

    int loops = 0;
    boolean in = false;
    boolean pressed = false;
    IMU imu;
    YawPitchRollAngles orientation;

    double newTarget;
    double inverseTarget;


    double thetakp = 0.045; // 0.1
    double xkp = 0.3; // .3
    double ykp = 0.3; // .15
    double xKv = 0.65; //.4
    double yKv = 0.65;
    double thetaKv = 0.05; // velocity constant

    double botProfileVelocity = 0;
    double botProfileAcceleration = 0;

    double aprilTagError = 0;

    double unboundedPower;
    int state = 0;

    double thetaCommand = 0;
    double xCommand = 0;
    double yCommand = 0;
    double botProgressCommand = 0;

    double xPositionSlope = 0;
    double yPositionSlope = 0;
    double thetaPositionSlope = 0;

    double xStartPosition = 0;
    double yStartPosition = 0;
    double thetaStartPosition = 0;

    double snapshot;

    boolean includeTheta = true;
    boolean includex = true;
    boolean includey = true;
    boolean atBase=false;
    double slewSpeed = 1.2;
    int accelTime = 25; // num of loops to take when accelerating
    double halfTargetDistance = 0;


    double xTarget = 0;
    double yTarget = 0;
    double thetaTarget = 90;
    double botTarget = 0;
    double slideCountsPerInch = 82;

    double leftFrontPower = 0;
    double rightFrontPower = 0;

    double leftBackPower = 0;
    double rightBackPower = 0;

    double thetaLeftFrontPower = 0;
    double thetaRightFrontPower = 0;
    double thetaLeftBackPower = 0;
    double thetaRightBackPower = 0;
    double position=0;
    boolean pickupDone=false;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor hangArm = null;

    double autonDuration = 0;

    private ElapsedTime runtime = new ElapsedTime();

    double frontEncoderInitial = 0;
    double backEncoderInitial = 0;
    double perpInitial = 0;

    double thetaErrorNm1 = 0; //error 1 step before
    double thetaErrorN = 0; // current error
    double thetaVelocity = 0; //difference between current and previous error

    double xErrorNm1 = 0;
    double xErrorN = 0;
    double xVelocity = 0;
    boolean move2done=false;


    double yErrorNm1 = 0;
    double yErrorN = 0;
    double yVelocity = 0;

    double nfrontEncoder = 0;
    double nbackEncoder = 0;
    double nperp = 0;

    int numberShots = 3;

    double nm1frontEncoder = 0;
    double nm1backEncoder = 0;
    double nm1perp = 0;

    boolean redAlliance=true;
    boolean closePos=false;

    double deltafrontEncoder = 0;
    double deltabackEncoder = 0;
    double deltaperp = 0;

    double botx = 48;
    double boty = 9;

    double deltax = 0;
    double deltay = 0;

    double inchesPerCount = 0.002844488;
    double averagePar = 0;
    double thetaRadians = 0;

    double botDirectionToGo = 0;
    double powerVectorDirection = 0;
    double linearPower = 0;

    double xPower = 0;
    double yPower = 0;

    int delayCounter = 0;
    int moveCounter = 0;
    int pauseCounter = 0;
    int counter = 0;
    int waitCounter=0;

    int halfTime = 0;
    int halfSlewTime = 0;
    int exitDistanceCounter = 0;


    double thetaScale = 0.3;
    double powerCompensation = 0.03;

    double perpCompensation = 0;
    double perpRadius = 4; //distance from perp encoder

    double xThird = 0;
    double xSecond = 0;
    double xFirst = 0;
    double xZero = 0;

    double segmentDuration = 50;
    double xInitPosition = 0;
    double xInitVelocity = 0;
    double xInitAcceleration = 0;

    double xFinalPosition = 0;
    double xFinalVelocity = 0;
    double xFinalAcceleration = 0;

    double yThird = 0;
    double ySecond = 0;
    double yFirst = 0;
    double yZero = 0;
    double yInitPosition = 0;
    double yInitVelocity = 0;
    double yInitAcceleration = 0;
    double yFinalPosition = 0;
    double yFinalVelocity = 0;
    double yFinalAcceleration = 0;

    double thetaThird = 0;
    double thetaSecond = 0;
    double thetaFirst = 0;
    double thetaZero = 0;
    double thetaInitPosition = 0;
    double thetaInitVelocity = 0;
    double thetaInitAcceleration = 0;
    double thetaFinalPosition = 0;
    double thetaFinalVelocity = 0;
    double thetaFinalAcceleration = 0;

    double servoWaitCounter = 0;

    double kfactor = 2;

    int id = 0;

    int PVT_index = -1;
    int PVT_length = 12;

    //PVT Table
    double[] Xp = new double[PVT_length];
    double[] Yp = new double[PVT_length];
    double[] Tp = new double[PVT_length];

    double[] Xv = new double[PVT_length];
    double[] Yv = new double[PVT_length];
    double[] Tv = new double[PVT_length];

    int slackValue;
    int farSlackValue = ((-1100* 312)/435);
    int closeSlackValue= ((-625* 312)/435);

    int currentIndex = 1;
    int value = 25;
    double[] PVTsegmentDuration = new double[PVT_length];

    private DcMotor arm = null;
    private DcMotor slide = null;
    private Servo clawServo;
    private Servo gripServo;

    int tiltCountsPerDegree = 37;

    double alignmentPower = 0;

    String gripperMode= "external";

    String nextGripperMode = "external";

    int armDownCounter=0;

    int otherTeamWaitCount = 0;
    boolean secondSpec = true;

    double thetaOffset = 0;

    double xterm1 = 0;
    double xterm2 =0;
    double yterm1 = 0;
    double yterm2 = 0;
    double movespeed = 0.9;
    Servo catapultServo;
    DcMotor catapultMotor;
    DcMotor endgameMotor;
    TouchSensor test_touch;
    //color reads purple at mid to be 55 red 75 green 105 blue
    //reads green at 50 175 133
    ColorSensor color;
    // ColorSensor color2;
    DcMotor motor;
    // DcMotor catapultMotor;
    // Servo servoGate;
    DcMotor transfermotor;

    boolean red_color;
    boolean blue_color;
    boolean yellow_color;





    // Define a variable for our color sensor
    // Servo pushServoOuttake;



    //1 inch and a quarter
    int r = 0;
    int g = 0;
    int p = 0;
    int gr = 0;

    int ballCounter = 0;



    boolean isPurple;
    boolean isGreen;
    boolean catapultReady = true; //true

    boolean artifactReady = false;

    boolean doneLaunching = false;
    boolean odometryOverrride = true;

    //12959-12705=254
    int GreenSensorCounter = 0;
    int GreenLoops = 5;
    int PurpleSensorCounter = 0;
    int PurpleLoops = 5;
    int desiredArtifact = 1;
    double classifier;
    int stackDelay=0;
    boolean isMove4=false;

    boolean xlastButton=false;
    boolean ylastButton=false;
    boolean downLastButton=false;
    int motorPos=0;
    static final int noArtifact = 0;
    static final int greenArtifact = 1;
    static final int purpleArtifact = 2;
    // private DigitalChannel redLED1;
    // private DigitalChannel greenLED1;
    // private DigitalChannel redLED2;
    // private DigitalChannel greenLED2;
    int delay=0;
    int[] artifactOrder = new int[3]; //order as artifacts come in
    int artifactIndex = 0;

    boolean firstLoop=true;


    int[] patternOrder = new int[3];//order of the motif
    int patternIndex = 0; //what artifact we want

    double unlockPos = 0.65;
    double lockPos = 0.71; //0.56

    int motorReference = 0;//where you get the third ball motor position
    int miniStackTarget = 75;
    int stackTarget = 500; //1000 distance to make all artifacts be stacked together
    int zeroExitDistance = 0;
    int oneExitDistance = -280;//-260
    int twoExitDistance = -600;//-600

    int blinkCount=0;

    int[] resequence = new int[3];
    int setValue = 300;

    int launchWait = 0;
    int intakeToOuttakeLoops = 0;
    int offset = 0;

    boolean tracking = false;
    boolean endgameStart=false;
    double apriltagturnspeed = 0.04;
    double aprilTagVelocity = 0;
    double pollenVelocity = 0;
    double pollenError = 0;
    double pollenErrorNm1;
    double pollenturnspeed = 0.02;
    boolean pollenTracking = false;
    double aprilTagErrorNm1;
    double kAT = 0.02;
    double aprilTagOffset = 0;

    public class BotVector {
        public double xPos;
        public double yPos;
        public double thetaPos;
        public double xVel;
        public double yVel;
        public double thetaVel;
        public double loops;

        public void initVariables(double xPos, double yPos, double thetaPos, double xVel, double yVel, double thetaVel, double loops){
            this.xPos = xPos;
            this.yPos = yPos;
            this.thetaPos = thetaPos;
            this.xVel = xVel;
            this.yVel = yVel;
            this.thetaVel = thetaVel;
            this.loops = loops;
        }
    }


    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        BotVector obelisk = new BotVector();
        BotVector base = new BotVector();
        BotVector spike1a = new BotVector();
        BotVector spike1b = new BotVector();
        BotVector base2 = new BotVector();
        BotVector spike2a = new BotVector();
        BotVector spike2b = new BotVector();
        BotVector base3 = new BotVector();
        BotVector end = new BotVector();

        if(!redAlliance && !closePos) {
            base.initVariables(52, 18, 22, 0, 0, 0, 10*kfactor);
            // spike1a.initVariables(42, 35, 105, -0.35, 0.3, 0.1, 10*kfactor);
            spike1a.initVariables(35.03, 34, 115.72, -0.5, 0, 0, 5*kfactor);
            // spike1b.initVariables(12, 45, 106, 0, 0, 0, 28*kfactor);
            spike1b.initVariables(6, 34, 115.84, 0, 0, 0, 20*kfactor);
            // base2.initVariables(60, 21, 18, 0, 0, 0, 13*kfactor);
            base2.initVariables(60, 18, 25, 0, 0, 0, 13*kfactor);
            // spike2a.initVariables(52, 61.58, 100, 0, 0, 0, 20*kfactor);//120.08
            spike2a.initVariables(34, 45, 115.95, -0.5, 0, 0, 20*kfactor);
            // spike2b.initVariables(20, 68.4, 103, 0, 0, 0, 20*kfactor);//105
            spike2b.initVariables(6.04, 45, 115.95, 0, 0, 0, 20*kfactor);
            // base3.initVariables(60, 25, 18, 0, 0, 0, 15*kfactor);
            base3.initVariables(60, 18, 25, 0, 0, 0, 15*kfactor);
            end.initVariables(10, 25, 0, 0, 0, 0, 10*kfactor);
        } else if (redAlliance && closePos) {
            obelisk.initVariables(29.6, 8.5, 55.6, 0, 0, 0, 10*kfactor);
            base.initVariables(29.6, 8.5, -52.6, 0, 0, 0, 10*kfactor);
            spike1a.initVariables(28, -12, -69, 0, 0, 0, 10*kfactor);//70.2
            spike1b.initVariables(45.5, -12., -69, 0, 0, 0, 20*kfactor);
            base2.initVariables(29.6, 8.5, -52.6, 0, 0, 0, 10*kfactor);
            spike2a.initVariables(27, -32, -72.7, 0, 0, 0, 15*kfactor);
            spike2b.initVariables(42, -33, -72.5, 0, 0, 0, 20*kfactor);
            base3.initVariables(29.6, 8.5, -52.6, 0, 0, 0, 10*kfactor);
            end.initVariables(5, 15, 0, 0, 0, 0, 10*kfactor);
        } else if(!redAlliance&&closePos){
            obelisk.initVariables(77, 8.5, -55.6, 0, 0, 0, 10*kfactor);
            base.initVariables(76, 8.5, 49.6, 0, 0, 0, 5*kfactor);
            spike1a.initVariables(76, -21, 110.2, 0, 0, 0, 10*kfactor);
            spike1b.initVariables(52.5, -21, 110.2, 0, 0, 0, 20*kfactor);
            base2.initVariables(77.6, 8.5, 49.6, 0, 0, 0, 5*kfactor);
            spike2a.initVariables(85, -44, 110.7, 0, 0, 0, 15*kfactor);
            spike2b.initVariables(54.1, -45, 110.5, 0, 0, 0, 20*kfactor);
            base3.initVariables(77.6, 8.5, 49, 0, 0, 0, 5*kfactor);
            end.initVariables(53, 15, 0, 0, 0, 0, 10*kfactor);
        } else if (redAlliance&& !closePos) {
            //temp
            obelisk.initVariables(43.9, 24.4, 0, 0, 0, 0, 10*kfactor);
            base.initVariables(43, 24.4, -32, 0, 0, 0, 7*kfactor);
            spike1a.initVariables(55, 39, -73.2, 0, 0, 0, 5*kfactor);
            spike1b.initVariables(85, 38, -73.2, 0, 0, 0, 33*kfactor);
            // base.initVariables(39.9, 24.4, -32, 0, 0, 0, 10*kfactor);
            // spike1a.initVariables(60, 50, -73.2, 0, 0, 0, 10*kfactor);
            // spike1b.initVariables(82.7, 50, -73.2, 0, 0, 0, 20*kfactor);
            spike2a.initVariables(55, 59, -73, 0, 0, 0, 12*kfactor);
            spike2b.initVariables(87, 58.5, -73, 0.5, 0, 0, 33*kfactor);
            base3.initVariables(43,24.4,-32,0,0,0,10*kfactor);
            end.initVariables(50, 27, -90, 0, 0, 0, 10*kfactor);
        }



        //old stuff:
        // if (redAlliance) {
        //     base.initVariables(60, 18, 18, 0, 0, 0, 10*kfactor);
        // }
        // else {
        //             base.initVariables(60, 18, 25, 0, 0, 0, 10*kfactor);
        // spike1a.initVariables(42, 35, 105, -0.35, 0.3, 0.1, 10*kfactor);
        // spike1a.initVariables(35.03, 34, 115.72, -0.5, 0, 0, 5*kfactor);
        // // spike1b.initVariables(12, 45, 106, 0, 0, 0, 28*kfactor);
        // spike1b.initVariables(6, 34, 115.84, 0, 0, 0, 20*kfactor);
        // // base2.initVariables(60, 21, 18, 0, 0, 0, 13*kfactor);
        // base2.initVariables(60, 18, 25, 0, 0, 0, 13*kfactor);
        // // spike2a.initVariables(52, 61.58, 100, 0, 0, 0, 20*kfactor);//120.08
        // spike2a.initVariables(37, 50, 115.95, -0.5, 0, 0, 20*kfactor);
        // // spike2b.initVariables(20, 68.4, 103, 0, 0, 0, 20*kfactor);//105
        // spike2b.initVariables(10.04, 51, 115.95, 0, 0, 0, 20*kfactor);
        // // base3.initVariables(60, 25, 18, 0, 0, 0, 15*kfactor);
        // base3.initVariables(60, 18, 30, 0, 0, 0, 15*kfactor);
        // end.initVariables(24, 10, 90, 0, 0, 0, 5*kfactor);base.initVariables(60, 18, 18, 0, 0, 0, 10*kfactor);
        // }

        if (isAuton){
            movePattern = MovePattern.moveStart;
            intakeState = IntakeState.done;
        }

        xCommand = botx;
        yCommand = boty;
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        catapultServo = hardwareMap.get(Servo.class, "servo1");
        // servoGate = hardwareMap.get(Servo.class, "servoGate");
        catapultMotor = hardwareMap.get(DcMotor.class, "catapult");
        test_touch = hardwareMap.get(TouchSensor.class, "touch");
        transfermotor = hardwareMap.get(DcMotor.class, "transferMotor");
        // redLED1 = hardwareMap.get(DigitalChannel.class, "red1");
        // greenLED1 = hardwareMap.get(DigitalChannel.class, "green1");
        // redLED2 = hardwareMap.get(DigitalChannel.class, "red2");
        // greenLED2 = hardwareMap.get(DigitalChannel.class, "green2");


        color = hardwareMap.get(ColorSensor.class, "color");
        // color2 = hardwareMap.get(ColorSensor.class, "color2");

        motor = hardwareMap.get(DcMotor.class, "intake");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // limelight.pipelineSwitch(7);
        // limelight.start();
        // arm = hardwareMap.get(DcMotor.class, "arm");
        // hangArm = hardwareMap.get(DcMotor.class, "hangArm");
        // slide = hardwareMap.get(DcMotor.class, "slide");
        // clawServo = hardwareMap.get(Servo.class, "clawServo");
        // gripServo = hardwareMap.get(Servo.class, "gripServo");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        endgameMotor = hardwareMap.get(DcMotor.class, "endgame");
        endgameMotor.setDirection(DcMotor.Direction.FORWARD);

        endgameMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // slide.setDirection(DcMotor.Direction.REVERSE);
        // slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // slide.setTargetPosition(0);
        // slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // slide.setPower(0);
        if(!closePos||!isAuton){
            slackValue=farSlackValue;
        }else{
            slackValue=closeSlackValue;
        }


        frontEncoderInitial = -leftFrontDrive.getCurrentPosition();
        backEncoderInitial = -leftBackDrive.getCurrentPosition();
        perpInitial = -rightFrontDrive.getCurrentPosition();

        botProfileAcceleration =  slewSpeed/accelTime;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // catapultServo.setPosition(0.67);
        // redLED1.setState(true);
        // greenLED1.setState(true);

        catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultMotor.setDirection(DcMotor.Direction.FORWARD);
        // catapultMotor.setPower(0.2);

        // // catapultServo.setPosition(unlockPos);
        // // sleep(1000);
        // catapultServo.setPosition(lockPos); //locked pos

        // // catapultServo.setPosition(0.5); //unlocked pos
        // catapultMotor.setTargetPosition((5000* 312)/435);
        // catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // while (!test_touch.isPressed() ){
        //     telemetry.addData("button press?: ", pressed);
        //     telemetry.addData("target position: ", catapultMotor.getTargetPosition());
        //     telemetry.update();
        // }
        // pressed = true;
        // telemetry.addData("button press?: ", pressed);
        // telemetry.update();
        // catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // if(!closePos||!isAuton){
        //     catapultMotor.setTargetPosition(farSlackValue);
        //     slackValue=farSlackValue;
        // }else{
        //     catapultMotor.setTargetPosition(closeSlackValue);
        //     slackValue=closeSlackValue;
        // }
        // catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // while (intakeState == IntakeState.hasOne) {
        //     redLED1.setState(false);
        //     greenLED1.setState(true);
        // }




        // if (motor.getPower()==0) {
        //     redLED1.setState(true);
        //     greenLED1.setState(false);
        // }

        // if (motor.getPower()!=0) {
        //     redLED1.setState(false);
        //     greenLED1.setState(true);
        // }

        // redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        // greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        // redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        // greenLED2.setMode(DigitalChannel.Mode.OUTPUT);

        // if(Math.abs(catapultMotor.getCurrentPosition()-slackValue)<20){
        //     redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        //     greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        //     redLED1.setState(false);
        //     greenLED1.setState(true);
        //     sleep(1000);
        //     // telemetry.addData("done with slack", (catapultMotor.getCurrentPosition()-slackValue)<20);
        // }

        // LLStatus status = limelight.getStatus();
        //     telemetry.addData("Name", "%s",
        //             status.getName());
        //     telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
        //             status.getTemp(), status.getCpu(),(int)status.getFps());
        //     telemetry.addData("Pipeline", "Index: %d, Type: %s",
        //             status.getPipelineIndex(), status.getPipelineType());

        //     LLResult result = limelight.getLatestResult();
        //     if (result.isValid()) {

        //         List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        //         for (LLResultTypes.DetectorResult dr : detectorResults) {
        //             telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
        //         }

        //     } else {
        //         telemetry.addData("Limelight", "No data available");
        //     }

        // LLResult resultinit = limelight.getLatestResult();
        // if (resultinit != null && resultinit.isValid()) {
        //     for (FiducialResult fiducial : resultinit.getFiducialResults()) {
        //         int id = fiducial.getFiducialId();  // <-- your AprilTag ID
        //         // telemetry.addData("ID", id);
        //         // telemetry.update();
        //     }
        // }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // telemetry.setAutoClear(false);
            if (!redAlliance) {
                aprilTagOffset = 2;
            } else {
                aprilTagOffset = 5;
            }
            if(firstLoop){
                catapultLaunchState = CatapultLaunchState.initArmBack;
                firstLoop=false;
            }

            if(!isAuton&& gamepad2.right_bumper){
                slackValue=closeSlackValue;
                catapultLaunchState = CatapultLaunchState.goToSlack;
            }
            if(!isAuton&&gamepad2.right_trigger!=0){
                slackValue=farSlackValue;
                catapultLaunchState = CatapultLaunchState.goToSlack;
            }


            // if(gamepad2.dpad_down && !downLastButton){
            //     numberShots++;
            //     if (numberShots == 1) {
            //         shotsLed = ShotsLed.oneShot;
            //         patternIndex = 2;
            //     } else if (numberShots == 2) {
            //         shotsLed = ShotsLed.twoShot;
            //         patternIndex = 1;
            //     } else if (numberShots == 3) {
            //         shotsLed = ShotsLed.threeShot;
            //         patternIndex = 0;
            //     } else if (numberShots == 4) {
            //         numberShots = 1;
            //         shotsLed = ShotsLed.oneShot;
            //         patternIndex = 2;
            //     }
            // }
            // downLastButton=gamepad2.dpad_down;

            // if(gamepad1.x && !xlastButton){
            //     artifactReady=true;
            // }
            // xlastButton=gamepad1.x;


            if((botx-base.xPos<=1 && boty-base.yPos<=1 && thetaPos()-base.thetaPos<=2)||(botx-base2.xPos<=1 && boty-base2.yPos<=1 && thetaPos()-base2.thetaPos<=2)||(botx-base3.xPos<=1 && boty-base3.yPos<=1 && thetaPos()-base3.thetaPos<=2)){
                atBase=true;
            }else{
                atBase=false;
            }
            if (!isAuton){

                // if(gamepad2.right_bumper){
                //     transfermotor.setTargetPosition(transfermotor.getCurrentPosition()+40);
                //     transfermotor.setPower(1);

                //     transfermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // }



                if(gamepad2.dpad_up){
                    transfermotor.setPower(-1);
                }else{
                    transfermotor.setPower(0);
                }

                if (gamepad1.b){
                    movespeed = 0.9;
                }
                if (gamepad1.y){
                    movespeed = 0.3;
                }

                if(gamepad1.left_trigger>0){
                    tracking=false;
                }
                if (gamepad1.right_trigger>0){
                    tracking = true;
                }

                if(gamepad1.left_bumper){
                    pollenTracking=false;
                }
                if (gamepad1.right_bumper){
                    pollenTracking = true;
                }


                if(gamepad2.dpad_left){
                    catapultLaunchState = CatapultLaunchState.servoRelease;
                }
                // if(gamepad2.)

                // xCommand += Math.pow(gamepad1.left_stick_x, 3)*movespeed;
                // yCommand -= Math.pow(gamepad1.left_stick_y, 3)*movespeed;
                double max;
                double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral =  -gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

                if (tracking){
                    yaw += (aprilTagError*apriltagturnspeed + aprilTagVelocity*kAT);
                }

                if (pollenTracking){
                    yaw = (pollenError*pollenturnspeed + pollenVelocity*kAT);
                }
                telemetry.addData("pollenError", pollenError);
                telemetry.addData("pollenErrorNm1", pollenErrorNm1);
                telemetry.addData("pollenVelocity", pollenVelocity);
                telemetry.addData("kAT", kAT);
                telemetry.addData("yaw", yaw);


                double frontLeftPower  = axial + lateral + yaw;
                double frontRightPower = axial - lateral - yaw;
                double backLeftPower   = axial - lateral + yaw;
                double backRightPower  = axial + lateral - yaw;



                max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
                max = Math.max(max, Math.abs(backLeftPower));
                max = Math.max(max, Math.abs(backRightPower));



                if (max > 1.0) {
                    frontLeftPower  /= max;
                    frontRightPower /= max;
                    backLeftPower   /= max;
                    backRightPower  /= max;
                }

                telemetry.addData("frontLeftPower",frontLeftPower);
                leftFrontDrive.setPower(frontLeftPower*movespeed);
                rightFrontDrive.setPower(frontRightPower*movespeed);
                leftBackDrive.setPower(backLeftPower*movespeed);
                rightBackDrive.setPower(backRightPower*movespeed);
                // if (gamepad1.right_stick_x!=0){
                //     thetaCommand += Math.pow(gamepad1.right_stick_x, 3)*5;
                // }
                if(gamepad2.dpad_right){
                    intakeState = IntakeState.miniStacking;
                }
                if (gamepad2.a) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    intakeState = IntakeState.empty;
                }
//                if(gamepad1.left_bumper&&gamepad1.right_bumper){
//                    endgameStart=true;
                    // leftFrontDrive.setPower(0);
                    // rightFrontDrive.setPower(0);
                    // leftBackDrive.setPower(0);
                    // rightBackDrive.setPower(0);
//                }

                if (gamepad1.dpad_up&&endgameStart){
                    endgameMotor.setTargetPosition(3000);
                    endgameMotor.setPower(0.2);
                    endgameMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad1.dpad_down&&endgameStart){
                    endgameMotor.setTargetPosition(400);
                    endgameMotor.setPower(0.2);
                    endgameMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }

                if(gamepad1.dpad_left&&endgameStart){
                    endgameMotor.setTargetPosition(endgameMotor.getCurrentPosition()+100);
                    endgameMotor.setPower(0.2);
                    endgameMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(gamepad1.dpad_right&&endgameStart){
                    endgameMotor.setTargetPosition(endgameMotor.getCurrentPosition()-100);
                    endgameMotor.setPower(0.2);
                    endgameMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                // if (gamepad2.right_bumper) {
                //     if (!test_touch.isPressed() ) {
                //         catapultMotor.setPower(0.7);
                //     }else{
                //       catapultMotor.setPower(0);
                //     }
                // }


                // while (gamepad2.right_bumper) {
                //      if (!test_touch.isPressed() ){
                //          catapultMotor.setPower(0.7);
                //      }else{
                //         catapultMotor.setPower(0);
                //      }
                // }

                if(gamepad1.a){
                    thetaOffset = thetaPos() +thetaOffset;
                    xCommand=botx;
                    yCommand=boty;
                    thetaCommand = 0;
                    tracking = true;
                }

                if(gamepad2.left_bumper) {
                    motor.setTargetPosition(motor.getCurrentPosition()-300);
                    motor.setPower(1);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(Math.abs(motor.getCurrentPosition()-motor.getTargetPosition()) < 10){
                        intakeState=IntakeState.miniStacked;
                    }
                }

                if(gamepad1.x) {
                    red_color = true;
                }
                if (gamepad1.y) {
                    blue_color = true;
                }
                if (gamepad1.a) {
                    yellow_color = true;
                }
                //test

                // if(intakeState==IntakeState.)



            }


            // if(gamepad1.dpad_up){
            //     catapultServo.setPosition(unlockPos);
            // }
            // if(gamepad1.dpad_down){
            //     catapultServo.setPosition(lockPos);//0.67
            // }



            // redLED1.setState(false);
            // clearPVTtable();

            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;
            // motor.setPower(1);
            // catapultMotor.setPower(1);



            double min;
            // telemetry.addData("movepattern state", movePattern);
            // telemetry.addData("curvestate ", curveState);
            // telemetry.addData("servo wait counter ", servoWaitCounter);
            // telemetry.addData("perp", perpPosition());

            // telemetry.addData("botx", botx);
            // telemetry.addData("boty", boty);
            // telemetry.addData("angle", thetaPos());
            // telemetry.addData("thetacommand", thetaCommand);


            // telemetry.addData("xError", xError());
            // telemetry.addData("yError", yError());
            // telemetry.addData("theta error: ", thetaError());


            // telemetry.addData("curve state: ", curveState);

            // telemetry.addData("move state: ", movePattern);
            // telemetry.addData("pvt segment duration", PVTsegmentDuration[currentIndex]);

            // telemetry.addData("move pattern state", movePattern);
            // telemetry.addData("hang arm", hangArm.getTargetPosition());
            // telemetry.addData("slide", slide.getTargetPosition());
            // telemetry.addData("arm target", arm.getTargetPosition());
            // telemetry.addData("arm current", arm.getCurrentPosition());
            // telemetry.addData("curve state", curveState);
            // telemetry.addData("wait state", waitState);
            // telemetry.addData("servo wait counter", servoWaitCounter);
            // telemetry.addData("time", autonDuration);
            // telemetry.addData("front encoder: ", frontEncoderPosition());
            // telemetry.addData("back encoder: ", backEncoderPosition());
            // telemetry.addData("motor target: ", motor.getTargetPosition());
            // telemetry.addData("xterm1 ", xterm1);
            // telemetry.addData("xterm2 ", xterm2);
            // telemetry.addData("yterm1 ", yterm1);
            // telemetry.addData("yterm2 ", yterm2);

            // telemetry.addData("front encoder")

//front 22, -1726 back 1070, -665, angle 40


            // telemetry.update();
            int r = color.red();
            int g = color.green();
            int b = color.blue();
            // int r2 = color2.red();
            // int g2= color2.green();
            // int b2= color2.blue();
            // LLResult result = limelight.getLatestResult();





            // Decide what color we see
            String detectedColor = "Unknown";
            // String detectedColor2 = "Unknown";
            //1 inch 60 red 300 green 250 blue
            //0.5 inch 100 red 500 green 300 blue
            // 600 red 2500 green 2000 blue
            //1300 red 5000 green 3700 blue


            //purp 1 inch 150 red 150 green 250 blue
            //0.5 inch 400 red 400 green 750 blue
            //0.25 inch 1800 red 1900 green 3400 blue
            //2400 2500 4300
            // telemetry.addData("Red", r);
            // telemetry.addData("Green", g);
            // telemetry.addData("Blue", b);
            // telemetry.addData("Detected Color", detectedColor);
            // telemetry.addData("Detected Color2", detectedColor2);
            aprilTagError = 0;
//            pollenError = 0;

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {

                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());

                    telemetry.addData("DetectorX", dr.getTargetXDegrees());
                    telemetry.addData("DetectorY", dr.getTargetYDegrees());
//                    classifier=dr.getTargetXDegrees();
//                    thetaCommand+=(classifier*-1);
                    pollenError = -dr.getTargetXDegrees();


//                    telemetry.addData("thetaCommand", thetaCommand);

                    //test test
                    //test
                }

            } else {
                telemetry.addData("Limelight", "No data available");
            }


            // if (result != null) {
            //     if (result.isValid()) {
            //         Pose3D botpose = result.getBotpose();
            //         // telemetry.addData("tx", result.getTx());
            //         // telemetry.addData("ty", result.getTy());
            //         if (result != null && result.isValid()) {
            //             for (FiducialResult fiducial : result.getFiducialResults()) {
            //                 int id = fiducial.getFiducialId();  // <-- your AprilTag ID
            //                 // telemetry.addData("ID", id);


            //                 if (((id==20 && (!redAlliance)) || (id==24 && redAlliance))&&!closePos){
            //                     aprilTagError = -result.getTx()-aprilTagOffset;
            //                 } else if (id == 21){
            //                     patternOrder[0] = 1;
            //                     patternOrder[1] = 2;
            //                     patternOrder[2] = 2;
            //                 } else if (id == 22){
            //                     patternOrder[0] = 2;
            //                     patternOrder[1] = 1;
            //                     patternOrder[2] = 2;
            //                 } else if (id == 23){
            //                     patternOrder[0] = 2;
            //                     patternOrder[1] = 2;
            //                     patternOrder[2] = 1;
            //                 } else if (id==20 && !redAlliance){
            //                   classifier=result.getTx();
            //                 } else if (id==24 && redAlliance) {
            //                     classifier=result.getTx();
            //                 }
            //               //telemetry.update();
            //             }


            //         }


            //     } else {
            //         aprilTagError = 0;
            //     }
            // }


            aprilTagVelocity = aprilTagError - aprilTagErrorNm1;
            aprilTagErrorNm1 = aprilTagError;

            pollenVelocity = pollenError - pollenErrorNm1;
            pollenErrorNm1 = pollenError;


            //color reads purple at mid to be 55 red 75 green 105 blue
            //purple hole 60 79 117
            //reads green at 50 175 133
            //green 35 122 90
            //33 100 77
            // Simple thresholds — you’ll likely want to tune these by testing
            if (g > 2*r && g > b ) {
                detectedColor = "Green";
                GreenSensorCounter ++;
            } else {
                GreenSensorCounter--;
            }

            // if (g2 > 2*r2 && g2 > b2) {
            //     detectedColor2 = "Green";

            // }


            if (GreenSensorCounter > GreenLoops) {
                GreenSensorCounter = GreenLoops;
            }
            if (GreenSensorCounter < 0) {
                GreenSensorCounter = 0;
            }
            if (GreenSensorCounter == 2) {
                isGreen = true;
            }
            if (GreenSensorCounter == 0) {
                isGreen = false;
            }
            //color reads purple at mid to be 55 red 75 green 105 blue
            //purple hole 60 79 117
            if (b > g && b > r*1.2) {
                // Purple is a mix of red + blue, with less green
                detectedColor = "Purple";
                PurpleSensorCounter++;
            } else {
                PurpleSensorCounter--;
            }

            // if (b2 > g2 && b2 > r2*1.2) {
            //     // Purple is a mix of red + blue, with less green
            //     detectedColor2 = "Purple";
            // }


            if (PurpleSensorCounter > PurpleLoops) {
                PurpleSensorCounter = PurpleLoops;
            }
            if (PurpleSensorCounter < 0) {
                PurpleSensorCounter = 0;
            }
            if (PurpleSensorCounter == 2) {
                isPurple = true;
            }
            if (PurpleSensorCounter == 0) {
                isPurple = false;
            }





            switch (artifactDetector){
                case start:
                    if (detectedColor.equals("Unknown")) {
                        artifactDetector = ArtifactDetector.noartifact;
                    }
                    break;


                case noartifact:
                    if (isPurple && artifactIndex < 3) {
                        p++;
                        artifactOrder[artifactIndex++] = purpleArtifact;
                        artifactDetector = ArtifactDetector.purple;
                    }
                    if (isGreen && artifactIndex < 3) {
                        gr++;
                        artifactOrder[artifactIndex++] = greenArtifact;
                        artifactDetector = ArtifactDetector.green;
                    }

                    // if((detectedColor2 == "green" && g==1) || (detectedColor2 == "purple" && p==2)){
                    //     // servoGate.setPosition(1);
                    // }
                    break;


                case purple:
                    if (isGreen && artifactIndex < 3) {
                        gr++;
                        artifactOrder[artifactIndex++] = greenArtifact;
                        artifactDetector = ArtifactDetector.green;
                    } else if (!isPurple) {
                        artifactDetector = ArtifactDetector.noartifact;
                    }
                    break;


                case green:
                    if (isPurple && artifactIndex < 3) {
                        p++;
                        artifactOrder[artifactIndex++] = purpleArtifact;
                        artifactDetector = ArtifactDetector.purple;
                    } else if (!isGreen) {
                        artifactDetector = ArtifactDetector.noartifact;
                    }
                    break;
            }





            switch (intakeState) {
                case ready:
                    // if(gamepad2.dpad_left){
                    //     catapultLaunchState = CatapultLaunchState.servoRelease;
                    // }
                    break;

                case empty:
                    motor.setPower(1);
                    // motorLed = MotorLed.isRunning;
                    if (gr+p == 1) {
                        intakeState = IntakeState.hasOne;
                    }
                    break;

                case hasOne:
                    // if (gr > 0){

                    // }

                    if (gr+p == 2) {
                        intakeState = IntakeState.hasTwo;
                    }
                    break;

                case hasTwo:

                    if (gr+p == 3) {
                        motorReference = motor.getCurrentPosition();
                        intakeState = IntakeState.hasThree;
                    }
                    break;

                case hasThree:
                    // servoGate.setPosition(1);

                    if (motor.getCurrentPosition() >= (motorReference + miniStackTarget)) {
                        motor.setPower(0);
                        motorReference = motor.getCurrentPosition();
                        intakeState = IntakeState.miniStacking;
                    }
                    break;

                case miniStacking:
                    motorReference = motor.getCurrentPosition();
                    // if(gamepad2.dpad_left && !isAuton){
                    //     motorPos += (40);
                    //     motor.setTargetPosition(motor.getCurrentPosition()+motorPos);
                    //     motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //     motor.setPower(1);
                    // }else
                    if (!isAuton){
                        motor.setPower(0);
                    }

                    // if(gamepad2.left_bumper) {
                    //     motor.setTargetPosition(motor.getCurrentPosition()-300);
                    // }
                    // motorLed = MotorLed.notRunning;
                    if (gamepad2.y || isAuton) {
                        motor.setPower(1);
                        intakeState = IntakeState.miniStacked;
                        // if (result != null) {
                        //     if (result.isValid()) {

                    }
                    //     }
                    // }
                    break;

                case miniStacked:


                    if (motor.getCurrentPosition() >= (motorReference + stackTarget)) {
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        intakeState = IntakeState.stacked;
                    }
                    break;

                case stacked:

                    // if(redAlliance){
                    //     if(!isAuton && botx>72 && catapultMotor.getCurrentPosition()<((-862.5* 312)/435)){
                    //         catapultMotor.setTargetPosition(closeSlackValue);
                    //         intakeState = IntakeState.waitForSlackChange;
                    //     } else if(!isAuton && botx<72 && catapultMotor.getCurrentPosition()>((-862.5* 312)/435)){
                    //         catapultMotor.setTargetPosition(farSlackValue);
                    //         intakeState = IntakeState.waitForSlackChange;
                    //     }
                    // }
                    // if(!redAlliance){
                    //     if(!isAuton && botx<-60 && catapultMotor.getCurrentPosition()<((-862.5* 312)/435)){
                    //         catapultMotor.setTargetPosition(closeSlackValue);
                    //         intakeState = IntakeState.waitForSlackChange;
                    //     } else if(!isAuton && botx>-60 && catapultMotor.getCurrentPosition()>((-862.5* 312)/435)){
                    //         catapultMotor.setTargetPosition(farSlackValue);
                    //         intakeState = IntakeState.waitForSlackChange;
                    //     }
                    // }

                    // if (numberShots == 1) {
                    //     shotsLed = ShotsLed.oneShot;
                    //     patternIndex = 2;
                    // } else if (numberShots == 2) {
                    //     shotsLed = ShotsLed.twoShot;
                    //     patternIndex = 1;
                    // } else if (numberShots == 3) {
                    //     shotsLed = ShotsLed.threeShot;
                    //     patternIndex = 0;
                    // }

                    if (gamepad2.b||(isAuton && patternOrder[1] != 0)) {
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        // LLResult result = limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            for (FiducialResult fiducial : result.getFiducialResults()) {
                                int id = fiducial.getFiducialId();  // <-- your AprilTag ID
                                // telemetry.addData("ID", id);
                                if ((redAlliance && id == 24)||(!redAlliance&&id==20)){
                                    thetaCommand+=(classifier*-1);
                                    // aprilTagError = result.getTx();
                                    // clearPVTtable();
                                    // appendPVTtable(botx, boty, (result.getTx()*-1)+thetaPos(), 0, 0, 0, 20);
                                    // movePath();

                                }
                            }
                            // telemetry.update();
                        }
                        // if (!isAuton){
                        //     intakeState = IntakeState.B;
                        // }
                        intakeState = IntakeState.calculateExitDistance;
                    }

                    if ((!isAuton&&gamepad2.x)||(patternOrder[1] == 0 && isAuton)){
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        // LLResult result = limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            for (FiducialResult fiducial : result.getFiducialResults()) {
                                int id = fiducial.getFiducialId();  // <-- your AprilTag ID
                                // telemetry.addData("ID", id);
                                if ((redAlliance && id == 24)||(!redAlliance&&id==20)){
                                    thetaCommand+=(classifier*-1);
                                    // clearPVTtable();
                                    // appendPVTtable(botx, boty, (result.getTx()*-1)+thetaPos(), 0, 0, 0, 20);
                                    // movePath();
                                }
                            }
                            // telemetry.update();
                        }
                        intakeState = IntakeState.B;
                    }
                    break;

                // case waitForSlackChange:
                //     if (Math.abs(catapultMotor.getCurrentPosition() - catapultMotor.getTargetPosition()) <= 10){
                //         intakeState = IntakeState.stacked;
                //     }
                //     break;
                case calculateExitDistance:
                    exitDistanceCounter++;

                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    offset = 0;
                    if (Arrays.equals(patternOrder, new int[]{2,2,1})) {

                        if (Arrays.equals(artifactOrder, new int[]{2,2,1})) {
                            // if (patternIndex==2){
                            //     offset=200;
                            // }
                            intakeState = IntakeState.D;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{2,1,2})) {
                            if (patternIndex==2){
                                offset=100;//usedto be 220
                            }
                            intakeState = IntakeState.E;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{1,2,2})) {
                            intakeState = IntakeState.B;
                        }

                    }
                    else if (Arrays.equals(patternOrder, new int[]{2,1,2})) {

                        if (Arrays.equals(artifactOrder, new int[]{2,2,1})) {
                            intakeState = IntakeState.A; //d
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{2,1,2})) {
                            intakeState = IntakeState.B;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{1,2,2})) {
                            if (patternIndex==2){
                                offset=100;
                            }
                            intakeState = IntakeState.E;
                        }

                    }
                    else if (Arrays.equals(patternOrder, new int[]{1,2,2})) {

                        if (Arrays.equals(artifactOrder, new int[]{2,2,1})) {
                            intakeState = IntakeState.B;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{2,1,2})) {
                            intakeState = IntakeState.A;
                        }
                        else if (Arrays.equals(artifactOrder, new int[]{1,2,2})) {
                            // if (patternIndex==1){
                            //     offset=120;
                            // }
                            // if (patternIndex==2){
                            //     offset=50;
                            // }
                            intakeState = IntakeState.C;
                        }
                    } else {
                        intakeState = IntakeState.B;
                    }
                    break;


                case E:
                    // resequence = new int[]{-setValue+50, 60, -2*setValue+210};
                    resequence = new int[]{0, -2*setValue+60, -setValue+60};
                    intakeState = IntakeState.setTarget;
                    break;

                case A:
                    resequence = new int[]{-setValue+30, 30, -2*setValue+130};
                    intakeState = IntakeState.setTarget;
                    break;

                case B:
                    // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resequence = new int[]{0, -setValue+70, (-2*setValue)+100}; //0, -setValue+100, (-2*setValue)+100
                    intakeState = IntakeState.setTarget;
                    break;

                case C:
                    resequence = new int[]{(-2 * setValue)+70, -setValue+100, 50};
                    intakeState = IntakeState.setTarget;
                    break;

                case D:
                    resequence = new int[]{-setValue+55, -2*setValue+120,60};
                    intakeState = IntakeState.setTarget;
                    break;

                // case X:
                //     resequence = new int[]{0, -setValue, 2 * setValue};
                //     intakeState = IntakeState.setTarget;
                //     break;

                case setTarget:
                    if (numberShots == 1) {
                        resequence = new int[] {0,0,(-2*setValue)+130};
                    } else if (numberShots == 2) {
                        resequence=new int[] {0,-setValue+55, -2*setValue+120};
                    }
                    position = resequence[patternIndex];
                    motor.setTargetPosition((int)position);
                    motor.setPower(1);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeState = IntakeState.waitForCatapult;
                    break;


                case waitForCatapult:
                    if(catapultReady){
                        intakeToOuttakeLoops = 0;
                        intakeState = IntakeState.intakeToOuttake;
                        catapultReady=false;
                    }
                    break;

                case intakeToOuttake:

                    if (Math.abs(motor.getCurrentPosition()-motor.getTargetPosition()) <= 20){
                        transfermotor.setPower(-1);
                        intakeToOuttakeLoops++;
                        if (intakeToOuttakeLoops > 20){
                            transfermotor.setPower(0);

                            artifactReady=true;

                            intakeState = IntakeState.presentNextArtifact;
                        }
                    }
                    break;

                case presentNextArtifact:
                    if (patternIndex>=2){
                        intakeState = IntakeState.done;
                    } else {
                        patternIndex++;
                        intakeState = IntakeState.setTarget;
                    }
                    break;

                // case waitForLaunch:
                // if (patternIndex == 3 && Math.abs(catapultMotor.getCurrentPosition()-catapultMotor.getTargetPosition()) < 30){
                //     intakeState = IntakeState.done;
                // } else {
                // intakeState = IntakeState.calculateExitDistance;
                // }

                // break;
                // case presentNextArtifact:
                //         if (patternIndex >= 2 && Math.abs(catapultMotor.getCurrentPosition()-catapultMotor.getTargetPosition()) < 30){
                //             intakeState = IntakeState.done;
                //         } else {
                //             patternIndex++;
                //             catapultLaunchState = catapultLaunchState.ready;
                //             intakeState = IntakeState.calculateExitDistance;
                //         }




                //     break;

                // case waitForLaunch:
                //     if (catapultLaunchState == catapultLaunchState.done) {
                //         if (patternIndex == 3 && Math.abs(catapultMotor.getCurrentPosition()-catapultMotor.getTargetPosition()) < 30){
                //             intakeState = IntakeState.done;
                //         } else {
                //             intakeState = IntakeState.calculateExitDistance;
                //         }
                //     }
                //     break;

                // case waitForLaunch:
                //     if (patternIndex == 3 && Math.abs(catapultMotor.getCurrentPosition()-catapultMotor.getTargetPosition()) < 30){
                //             intakeState = IntakeState.done;
                //         } else {
                //             intakeState = IntakeState.calculateExitDistance;
                //         }
                //     break;

                case done:
                    // pushServoOuttake.setPosition(0);
                    // catapultMotor.setTargetPosition(0);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor.setPower(0);
                    artifactIndex = 0;
                    patternIndex=0;
                    artifactOrder[0] = 0;
                    artifactOrder[1] = 0;
                    artifactOrder[2] = 0;
                    gr = 0;
                    p = 0;
                    if (!isAuton){
                        intakeState = IntakeState.ready;
                        artifactDetector = ArtifactDetector.start;

                    }
                    // intakeState = IntakeState.launch;
                    break;


                default:
                    throw new IllegalStateException("Unexpected value: " + intakeState);
            }

            //   switch (motorLed) {
            //       case ready:
            //           redLED1.setState(true);
            //           greenLED1.setState(true);
            //           break;

            //       case isRunning:
            //           redLED1.setState(true);
            //           greenLED1.setState(true);
            //           break;

            //         case notRunning:
            //             redLED1.setState(true);
            //             greenLED1.setState(false);
            //             break;

            //         case done:
            //             break;

            //   }

            //   switch(shotsLed) {
            //       case ready:
            //               redLED1.setState(true);
            //               greenLED1.setState(true);
            //               break;
            //         case oneShot:
            //             redLED2.setState(true);
            //             greenLED2.setState(true);
            //             break;

            //         case twoShot:
            //             blinkingState = BlinkingState.on;
            //             shotsLed=ShotsLed.done;
            //                 // blinkingState=BlinkingState.done;

            //             break;
            //         case threeShot:
            //             blinkingState = BlinkingState.done;
            //             redLED2.setState(false);
            //             greenLED2.setState(true);
            //             break;
            //         case done:
            //             break;

            //   }

            // switch (blinkingState){
            //     case on:
            //         redLED2.setState(false);
            //         greenLED2.setState(true);
            //         blinkCount++;
            //         if (blinkCount >= 5){
            //             blinkCount = 0;
            //             blinkingState = BlinkingState.off;
            //         }
            //         break;

            //     case off:
            //         redLED2.setState(true);
            //         greenLED2.setState(true);
            //         blinkCount++;
            //         if (blinkCount >= 5){
            //             blinkCount = 0;
            //             blinkingState = BlinkingState.on;
            //         }
            //         break;

            //     case done:
            //         break;
            // }

            switch (movePattern) {
                case moveStart:
                    movePattern = MovePattern.move1;
                    artifactOrder[0] = 2;
                    artifactOrder[1] = 1;
                    artifactOrder[2] = 2;
                    //   if (closePos) {
                    movePattern = MovePattern.obeliskView;
                    // }else {
                    //     movePattern = MovePattern.move1;
                    // }


                    break;

                case obeliskView:
                    clearPVTtable();
                    appendPVTtableBotVector(obelisk);//30.75
                    movePath();

                    movePattern = MovePattern.waitObeliskView;
                    break;

                case waitObeliskView:
                    if(curveState == CurveState.done){
                        movePattern = MovePattern.move1;
                    }
                    break;

                case move1:
                    // if(boty>13){
                    //     thetaTargetOverrride=true;
                    // }
                    clearPVTtable();
                    appendPVTtableBotVector(base);//30.75


                    movePath();
                    intakeState=IntakeState.stacked;
                    delay=0;
                    movePattern = MovePattern.waitmove1;

                    break;

                // case alignMove1:
                //     if(curveState == CurveState.done){
                //         clearPVTtable();
                //         appendPVTtable(60, 18, (-classifier+3), 0, 0, 0, 5*kfactor);//30.75
                //         movePath();
                //         // thetaTargetOverrride = false;
                //         movePattern = MovePattern.waitmove1;
                //     }

                case waitmove1:
                    // if(curveState == CurveState.done){

                    // atBase=true;
                    // }
                    // if (delay>=25 && intakeState == IntakeState.done && curveState==CurveState.done){
                    if (doneLaunching){
                        // atBase=false;
                        doneLaunching = false;
                        movePattern = MovePattern.move2;
                    }


                    break;

                case move2:
                    // inMove2=true;
                    move2done=true;
                    clearPVTtable();
                    appendPVTtableBotVector(spike1a);//100
                    appendPVTtableBotVector(spike1b);//105
                    movePath();
                    if (closePos) {
                        artifactOrder[0] = 2;
                        artifactOrder[1] = 2;
                        artifactOrder[2] = 1;
                    } else if (!closePos) {
                        artifactOrder[0] = 1;
                        artifactOrder[1] = 2;
                        artifactOrder[2] = 2;
                    }
                    intakeState = IntakeState.empty;
                    artifactDetector = ArtifactDetector.start;

                    movePattern = MovePattern.waitmove2;

                    break;

                case waitmove2:


                    if (curveState == CurveState.done){
                        pickupDone=true;
                        motorReference = motor.getCurrentPosition();
                        movePattern = MovePattern.move3;
                    }

                    break;


                case move3:
                    intakeState = intakeState.hasThree;
                    // if(boty<30){
                    //     thetaTargetOverrride=true;
                    // }
                    clearPVTtable();

                    appendPVTtableBotVector(base2);//30.75


                    movePath();


                    // clearPVTtable();
                    // appendPVTtable(48, 50, 135, 0, 0, 0, 50*kfactor);//30.75
                    // appendPVTtable(28, 50, 135, 0, 0, 0, 50*kfactor);//30.75
                    // appendPVTtable(60, 22, 20, 0, 0, 0, 50*kfactor);//30.75

                    // movePath();
                    movePattern = MovePattern.waitmove3;

                    break;

                // case alignMove3:
                //     if (curveState == CurveState.done){
                //         clearPVTtable();
                //         appendPVTtable(60, 21, (-classifier+3), 0, 0, 0, 5*kfactor);//30.75
                //         movePath();
                //         movePattern = MovePattern.waitmove3;
                //     }
                //     break;

                case waitmove3:

                    if (curveState == CurveState.done  && boty<25){
                        // atBase=true;

                        // thetaTargetOverrride=false;

                    }
                    if (doneLaunching) {
                        doneLaunching=false;
                        pickupDone=false;
                        movePattern = MovePattern.move4;
                    }

                    break;

                // case waitwaitmove3:
                //     if (waitState == WaitState.done){
                //         movePattern = MovePattern.move4;
                //     }
                // break;

                case move4:
                    // isMove4 = true;
                    clearPVTtable();

                    appendPVTtableBotVector(spike2a);//120.08
                    appendPVTtableBotVector(spike2b);//105
                    movePath();
                    artifactOrder[0] = 2;
                    artifactOrder[1] = 1;
                    artifactOrder[2] = 2;
                    intakeState = IntakeState.empty;
                    artifactDetector = ArtifactDetector.start;
                    movePattern = MovePattern.waitmove4;

                    break;

                case waitmove4:


                    if (curveState == CurveState.done){
                        pickupDone=true;

                        movePattern = MovePattern.move5;
                    }

                    break;

                case move5:
                    // thetaTargetOverrride=true;
                    clearPVTtable();
                    appendPVTtableBotVector(base3);//30.75

                    movePath();

                    // clearPVTtable();
                    // appendPVTtable(48, 50, 135, 0, 0, 0, 50*kfactor);//30.75
                    // appendPVTtable(28, 50, 135, 0, 0, 0, 50*kfactor);//30.75
                    // appendPVTtable(60, 22, 20, 0, 0, 0, 50*kfactor);//30.75

                    // movePath();
                    intakeState=IntakeState.miniStacking;
                    movePattern = MovePattern.waitmove5;

                    break;

                case waitmove5:
                    if (curveState == CurveState.done && boty<25){

                        // appendPVTtable(60, 21, (-classifier+3), 0, 0, 0, 5*kfactor);//30.75
                        // atBase=true;
                    }
                    if (doneLaunching) {
                        doneLaunching=false;
                        pickupDone=false;
                        movePattern = MovePattern.move6;
                    }

                    break;

                case move6:
                    clearPVTtable();
                    appendPVTtableBotVector(end);//30.75
                    movePath();
                    movePattern = MovePattern.done;
                    break;

                case done:
                    break;

                default:
                    movePattern = MovePattern.done;
            }
            switch (waitState) {
                case wait:
                    waitCounter++;
                    if (waitCounter>=100){
                        waitState = WaitState.done;
                    }
                    break;

                case done:
                    break;
            }

            switch(catapultLaunchState){

                case ready:
                    catapultLaunchState = CatapultLaunchState.waitForCatapultReady;
                    break;

                case initArmBack:
                    // catapultMotor.setPower(0.2);

                    catapultServo.setPosition(lockPos); //locked pos

                    catapultMotor.setTargetPosition((5000* 312)/435);
                    catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (test_touch.isPressed() ){
                        catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        catapultLaunchState = CatapultLaunchState.goToSlack;
                    }

                    break;

                case waitForCatapultReady:
                    loops++;
                    if(loops>=5){
                        catapultServo.setPosition(lockPos);
                    }
                    if (loops >= 10){
                        catapultMotor.setTargetPosition((100* 312)/435);//28
                        // catapultMotor.setPower(1);
                        // catapultServo.setPosition(lockPos);
                        catapultLaunchState = CatapultLaunchState.armBack;
                        armDownCounter=0;
                    }
                    break;

                case armBack:
                    catapultReady=true;
                    if ((test_touch.isPressed())){
                        armDownCounter++;
                    }else{
                        armDownCounter=0;
                    }
                    if(armDownCounter>=1){
                        armDownCounter=0;
                        // (Math.abs(catapultMotor.getCurrentPosition() - catapultMotor.getTargetPosition()) <= 36)||
                        catapultLaunchState = CatapultLaunchState.goToSlack;
                    }
                    break;

                // if ((Math.abs(catapultMotor.getCurrentPosition() - catapultMotor.getTargetPosition()) <= 36)||(test_touch.isPressed())){
                //     catapultLaunchState = CatapultLaunchState.goToSlack;
                // }
                // break;

                case goToSlack:
                    //   testdelay++;
                    // if (botx > 72 && !isAuton) {
                    //     slackValue = closeSlackValue;
                    // } else if (!isAuton) {
                    //     slackValue = farSlackValue;
                    // }
                    catapultMotor.setTargetPosition(slackValue);
                    catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (Math.abs(catapultMotor.getCurrentPosition() - catapultMotor.getTargetPosition()) <= 10){
                        // if(testdelay>=10){
                        catapultLaunchState =  CatapultLaunchState.waitForArtifactReady;
                        // }
                    }
                    break;

                case waitForArtifactReady:
                    if((artifactReady && atBase)|| (artifactReady&& !isAuton)){
                        catapultLaunchState = CatapultLaunchState.servoRelease;
                    }
                    break;

                case servoRelease:
                    catapultServo.setPosition(unlockPos);
                    ballCounter++;
                    if (ballCounter == 3){
                        doneLaunching = true;
                        ballCounter =0;
                    }
                    artifactReady=false;
                    loops = 0;
                    catapultLaunchState = CatapultLaunchState.ready;
                    break;


                case done:
                    break;
            }




            switch (profileState) {
                case accel:
                    if(botProgressCommand>=halfTargetDistance){ // check if we passed half the distance, triangle case
                        profileState = ProfileState.decel;
                    }
                    else if(botProfileVelocity<slewSpeed){ // keep accelerating
                        botProfileVelocity+=botProfileAcceleration;
                        // telemetry.addData("in state ", moveCounter);
                    }
                    else { // go to slew 1, stop accelerating
                        botProfileVelocity = slewSpeed;
                        profileState = ProfileState.slew1;
                    }
                    break;

                case slew1:
                    if(botProgressCommand>=halfTargetDistance){
                        halfTime = moveCounter;
                        profileState = ProfileState.slew2;
                        halfSlewTime = halfTime - accelTime;
                    }
                    break;

                case slew2:
                    if(moveCounter==halfSlewTime+halfTime){
                        profileState = ProfileState.decel;
                    }
                    break;

                case decel:
                    botProfileVelocity -= botProfileAcceleration;
                    if (botProfileVelocity < 0.001){
                        botProfileVelocity = 0.001;
                    }
                    if(botProgressCommand >= botTarget){
                        botProgressCommand = botTarget;
                        profileState = ProfileState.done;
                    }
                    break;

                case done:
                    botProfileVelocity = 0;
                    break;

                default:
                    profileState = ProfileState.done;
            }

            switch (curveState){
                case calculatingCoefficients:
                    calculateCoefficients(currentIndex);
                    curveState = CurveState.calculatingCommands;
                    break;

                case calculatingCommands:
                    if ((moveCounter > PVTsegmentDuration[currentIndex])){
                        currentIndex += 1;
                        if (currentIndex > PVT_index){
                            curveState = CurveState.done;
                        } else {
                            calculateCoefficients(currentIndex);


                        }
                    }

                    // && (Math.abs(xErrorN)<1) && (Math.abs(yErrorN)<1) && (Math.abs(thetaErrorN)<3)
                    // xCommand = ((xThird*Math.pow(moveCounter,3)) + (xSecond*Math.pow(moveCounter,2)) + (xFirst*moveCounter) + (xZero));
                    // yCommand = ((yThird*Math.pow(moveCounter,3)) + (ySecond*Math.pow(moveCounter,2)) + (yFirst*moveCounter) + (yZero));
                    if (odometryOverrride){
                        // double max;


                    }
                    // else{
                    //     thetaCommand = ((thetaThird*Math.pow(moveCounter,3)) + (thetaSecond*Math.pow(moveCounter,2)) + (thetaFirst*moveCounter) + (thetaZero));
                    // }
                    break;

                case done:

                    break;

                default:
                    curveState = CurveState.done;
            }



            // yCommand = (yTarget-yStartPosition)*(botProgressCommand/botTarget)+yStartPosition;
            // thetaCommand = (thetaTarget-thetaStartPosition)*(botProgressCommand/botTarget)+thetaStartPosition;

            if (botProgressCommand < botTarget){
                botProgressCommand += botProfileVelocity;
            }

            thetaErrorN = thetaError();
            thetaVelocity = thetaErrorN - thetaErrorNm1;

            thetaLeftFrontPower = -(thetaErrorN*thetakp + thetaVelocity*thetaKv);
            thetaRightFrontPower =(thetaErrorN*thetakp + thetaVelocity*thetaKv);
            thetaLeftBackPower = -(thetaErrorN*thetakp + thetaVelocity*thetaKv);
            thetaRightBackPower =(thetaErrorN*thetakp + thetaVelocity*thetaKv);

            thetaErrorNm1 = thetaErrorN;

            xErrorN = xError();
            xVelocity = xErrorN - xErrorNm1; // how much the error changed from current cycle to previous cycle.
            xPower = xErrorN*xkp + xVelocity * xKv;

            yErrorN = yError();
            yVelocity = yErrorN - yErrorNm1;
            yPower = yErrorN*ykp + yVelocity * yKv;

            xErrorNm1 = xErrorN;
            yErrorNm1 = yErrorN;

            // Theta is angle of robot on the field
            // Error is the direction the bot should travel to from theta=0
            // botDirectionToGo is the direction the bot should travel to from its current theta
            linearPower = Math.sqrt(xPower*xPower + yPower*yPower);
            powerVectorDirection = (Math.atan2(yPower,xPower)*(180/Math.PI)) -90;
            botDirectionToGo = powerVectorDirection - thetaPos();


            // double leftFrontPower  = hysteresisPower(Math.cos((botDirectionToGo +45)*(Math.PI/180))* linearPower + thetaLeftFrontPower);
            // double rightFrontPower = hysteresisPower(Math.cos((botDirectionToGo -45)*(Math.PI/180))* linearPower + thetaRightFrontPower);
            // double leftBackPower   = hysteresisPower(Math.cos((botDirectionToGo -45)*(Math.PI/180))* linearPower + thetaLeftBackPower);
            // double rightBackPower  = hysteresisPower(Math.cos((botDirectionToGo +45)*(Math.PI/180))* linearPower + thetaRightBackPower);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            // max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            // max = Math.max(max, Math.abs(leftBackPower));
            // max = Math.max(max, Math.abs(rightBackPower));

            // if (max > 1.0) {
            //     leftFrontPower  /= max;
            //     rightFrontPower /= max;
            //     leftBackPower   /= max;
            //     rightBackPower  /= max;
            // }
            //THIS MINUS SIGN EXISTS BECAUSE IF CHANGING DIRECTION AT START, messes up positive power being forward
            // if(!endgameStart){
            //     leftFrontDrive.setPower(-leftFrontPower);
            //     rightFrontDrive.setPower(-rightFrontPower);
            //     leftBackDrive.setPower(-leftBackPower);
            //     rightBackDrive.setPower(-rightBackPower);
            // }else{
            //     leftFrontDrive.setPower(0);
            //     rightFrontDrive.setPower(0);
            //     leftBackDrive.setPower(0);
            //     rightBackDrive.setPower(0);
            // }

            // telemetry.addData("lfpower", -leftFrontPower);
            // telemetry.addData("rfpower", -rightFrontPower);
            // telemetry.addData("lbpower", -leftBackPower);
            // telemetry.addData("rbpower", -rightBackPower);
            // telemetry.addData("botdirectiontogo ", botDirectionToGo);
            // telemetry.addData("waitCounter", waitCounter);

            // telemetry.addData("yErrorN", yErrorN);
            // telemetry.addData("xErrorN", xErrorN);
            // telemetry.addData("Red", r);
            // telemetry.addData("Green", g);
            // telemetry.addData("Blue", b);
            // telemetry.addData("Detected Color", detectedColor);
            // telemetry.addData("Detected Color2", detectedColor2);
            // telemetry.addData("theta Error", thetaError());
            // telemetry.addData("tx", (result.getTx()*-1));


            // telemetry.addData("purple counted", p);
            // telemetry.addData("green counted", gr);
            // telemetry.addData("motor2 current pos", catapultMotor.getCurrentPosition());

            // telemetry.addData("catapult ready", catapultReady);
            // telemetry.addData("artifact ready", artifactReady);
            // telemetry.addData("intake state", intakeState);
            // telemetry.addData("pattern index", patternIndex);
            // telemetry.addData("at base: ", atBase);
            // telemetry.addData("intake state", intakeState);
            // telemetry.addData("artifact detector state", artifactDetector);
            // telemetry.addData("motor reference", motorReference);
            // telemetry.addData("motor target pos", motor.getTargetPosition());
            // telemetry.addData("intakemotor current pos", motor.getCurrentPosition());
            // telemetry.addData("Artifact Order", Arrays.toString(artifactOrder));
            // telemetry.addData("pattern order: ", Arrays.toString(patternOrder));
            // telemetry.addData("motor ref+stacktarget: ", (motorReference + stackTarget));
            // telemetry.addData("classifier:", classifier);
            // telemetry.addData("delay>=25", delay >= 25);
            // telemetry.addData("intake done", intakeState == IntakeState.done);
            // telemetry.addData("curve done", curveState == CurveState.done);
            // telemetry.addData("move1done", move1done);
            // telemetry.addData("movePattern case:", movePattern);
            // telemetry.addData("slack value: ", slackValue);
            // telemetry.addData("current index", currentIndex);
            // telemetry.addData("firstloop", firstLoop);
            // telemetry.addData("artifact ready: ", artifactReady);
            // telemetry.addData("catapult state: ", catapultLaunchState);

            // telemetry.addData("ID:", result.getFiducialResults());
            // telemetry.addData("curveState done: ", curveState.equals(CurveState.done));
            // telemetry.addData("thetaPos: ", thetaPos());
            // telemetry.addData("theta error: ", thetaError());

            // telemetry.addData("snapshot", snapshot);
            // telemetry.addData("frontencoder", frontEncoderPosition());
            // telemetry.addData("backencoder", backEncoderPosition());
            // telemetry.addData("botx",botx);
            // telemetry.addData("boty",boty);
            // telemetry.addData("bot theta",thetaPos());
            // telemetry.addData("xcommand", xCommand);
            // telemetry.addData("yCommand", yCommand);
            // telemetry.addData("classifier", classifier);
            // telemetry.addData("numberShots: ", numberShots);
            // telemetry.addData("blinking state", blinkingState);
            // telemetry.addData("shotsLed state", shotsLed);
            // telemetry.addData("blink Count", blinkCount);


            // telemetry.a
            // telemetry.addData("pvt index:", PVT_index);
            telemetry.addData("pollentracking", pollenTracking);
            telemetry.update();



            moveCounter += 1;
            counter +=1;

            perpCompensation = -((perpRadius*2*Math.PI)*(thetaPos()/360))/inchesPerCount;

            nfrontEncoder = frontEncoderPosition();
            nbackEncoder = backEncoderPosition();
            nperp = perpPosition()-perpCompensation;

            deltafrontEncoder = nfrontEncoder - nm1frontEncoder;
            deltabackEncoder = nbackEncoder - nm1backEncoder;
            deltaperp = nperp - nm1perp;

            nm1frontEncoder = nfrontEncoder;
            nm1backEncoder = nbackEncoder;
            nm1perp = nperp;

            averagePar = (deltabackEncoder+deltafrontEncoder)/2;
            thetaRadians = thetaPos()*(Math.PI/180);

            xterm1 = -averagePar*Math.cos(thetaRadians);
            yterm1 = -averagePar*Math.sin(thetaRadians);
            xterm2 = deltaperp*Math.sin(thetaRadians);
            yterm2 = -deltaperp*Math.cos(thetaRadians);

            deltax = -averagePar*Math.cos(thetaRadians)  +  deltaperp*Math.sin(thetaRadians);
            deltay = -averagePar*Math.sin(thetaRadians) - deltaperp*Math.cos(thetaRadians);

            botx += deltax*inchesPerCount;
            boty += deltay*inchesPerCount;






            // telemetry.update();
            // if(value > 20 ){
            //     throw new RuntimeException("value is not high");

            // }

        }
    }

    public double frontEncoderPosition(){
        return -leftFrontDrive.getCurrentPosition() - frontEncoderInitial;
    }

    public double backEncoderPosition(){
        return leftBackDrive.getCurrentPosition() - backEncoderInitial;
    }

    public double perpPosition(){
        return -rightFrontDrive.getCurrentPosition() - perpInitial;
    }

    public void moveTo (double x, double y, double theta){
        // yTarget = y;
        // thetaTarget = theta;
        // yStartPosition = yCommand;
        // thetaStartPosition = thetaCommand;
        botTarget = 0;
        botProfileVelocity = 0;
        botProgressCommand = 0;
        halfTargetDistance = botTarget/2;
        xFinalPosition = x;
        yFinalPosition = y;
        thetaFinalPosition = theta;




        // xZero = xInitPosition;
        // xFirst = xInitVelocity;
        // xThird = (2*(xFinalPosition - xInitPosition - xInitVelocity*segmentDuration)-(xFinalVelocity-xInitVelocity)*Math.pow(segmentDuration,2))/Math.pow(segmentDuration,3);
        // xSecond = (xFinalVelocity - xInitVelocity - (3*xThird*Math.pow(segmentDuration,2)))/(2*segmentDuration);

        // yZero = yInitPosition;
        // yFirst = yInitVelocity;
        // yThird = (2*(yFinalPosition - yInitPosition - yInitVelocity*segmentDuration)-(yFinalVelocity-yInitVelocity)*Math.pow(segmentDuration,2))/Math.pow(segmentDuration,3);
        // ySecond = (yFinalVelocity - yInitVelocity - (3*yThird*Math.pow(segmentDuration,2)))/(2*segmentDuration);

        // thetaZero = thetaInitPosition;
        // thetaFirst = thetaInitVelocity;
        // thetaThird = (2*(thetaFinalPosition - thetaInitPosition - thetaInitVelocity*segmentDuration)-(thetaFinalVelocity-thetaInitVelocity)*Math.pow(segmentDuration,2))/Math.pow(segmentDuration,3);
        // thetaSecond = (thetaFinalVelocity - thetaInitVelocity - (3*thetaThird*Math.pow(segmentDuration,2)))/(2*segmentDuration);



        moveCounter = 0;
        profileState = ProfileState.accel;
    }

    public double boundedPower (double unboundedPower) {
        if (unboundedPower>1){
            return 1;
        }
        else if (unboundedPower<-1){
            return -1;
        }
        else {
            return unboundedPower;
        }
    }

    public double thetaPos(){
        return (frontEncoderPosition() - backEncoderPosition()) * 0.0160496469 - thetaOffset; // 90/8000 (0.01125) old robot
        //3600 = (260572)*x
        //x = 0.0138158 new robot
    }

    public double thetaError (){
        return thetaCommand-thetaPos();
    }

    public double xError(){
        return xCommand-botx; // -0.5 - 0 → error is -0.5
    }

    public double yError(){
        return yCommand-boty;
    }

    public boolean botHasReachedTarget(){
        if (Math.abs(botTarget - botProgressCommand) >= 0.001){
            return false;
        }
        return true;
    }

    public boolean botHasSettled(){
        if (!botHasReachedTarget()){
            delayCounter = 10;
            return false;
        }
        if (delayCounter > 0){
            delayCounter -= 1;
            return false;
        }
        return true;
    }

    public double hysteresisPower(double powerIn){
        if (powerIn > 0){
            return powerIn+powerCompensation;
        }
        else if (powerIn < 0){
            return powerIn-powerCompensation;
        }
        else {
            return 0;
        }
    }

    public void clearPVTtable(){
        PVT_index = -1;

        appendPVTtable(botx, boty, thetaPos(), 0, 0, 0, 0);
    }

    public void appendPVTtable(double Xp_p, double Yp_p, double Tp_p, double Xv_p, double Yv_p, double Tv_p, double PVTsegmentDuration_p){
        PVT_index ++;
        if (PVT_index > PVT_length-1){
            telemetry.addData("PVT Table index out of bounds!!", "):<");
        }
        else {
            Xp[PVT_index] = Xp_p;
            Yp[PVT_index] = Yp_p;
            Tp[PVT_index] = Tp_p;

            Xv[PVT_index] = Xv_p;
            Yv[PVT_index] = Yv_p;
            Tv[PVT_index] = Tv_p;
            PVTsegmentDuration[PVT_index] = PVTsegmentDuration_p;
        }

    }

    public void appendPVTtableBotVector(BotVector v) {
        appendPVTtable(v.xPos, v.yPos, v.thetaPos, v.xVel, v.yVel, v.thetaVel, v.loops);
    }

    public void calculateCoefficients(int N){
        moveCounter = 0;

        if (N<1){
            throw new RuntimeException("n is less than 1");
        }


        xInitPosition = Xp[N-1];
        xFinalPosition = Xp[N];
        xInitVelocity = Xv[N-1];
        xFinalVelocity = Xv[N];

        yInitPosition = Yp[N-1];
        yFinalPosition = Yp[N];
        yInitVelocity = Yv[N-1];
        yFinalVelocity = Yv[N];

        thetaInitPosition =  Tp[N-1];
        thetaFinalPosition = Tp[N];
        thetaInitVelocity = Tv[N-1];
        thetaFinalVelocity = Tv[N];

        segmentDuration = PVTsegmentDuration[N];

        xZero = xInitPosition ;
        xFirst =xInitVelocity ;
        xSecond = (3*(xFinalPosition - xInitPosition)/Math.pow(segmentDuration,2) - ((2*xInitVelocity) + xFinalVelocity)/segmentDuration);
        xThird = ((xFinalVelocity+xInitVelocity)/Math.pow(segmentDuration,2)) + ((2*(xInitPosition-xFinalPosition))/Math.pow(segmentDuration,3));

        // xThird = (2*(xFinalPosition - xInitPosition  - xInitVelocity*segmentDuration)-(xFinalVelocity-xInitVelocity)*Math.pow(segmentDuration,2))/Math.pow(segmentDuration,3);
        // xSecond = (xFinalVelocity - xInitVelocity - (3*xThird*Math.pow(segmentDuration,2)))/(2*segmentDuration);

        yZero = yInitPosition;
        yFirst = yInitVelocity;
        ySecond = (3*(yFinalPosition - yInitPosition)/Math.pow(segmentDuration,2) - ((2*yInitVelocity) + yFinalVelocity)/segmentDuration);
        yThird = ((yFinalVelocity+yInitVelocity)/Math.pow(segmentDuration,2)) + ((2*(yInitPosition-yFinalPosition))/Math.pow(segmentDuration,3));

        thetaZero = thetaInitPosition;
        thetaFirst = thetaInitVelocity;
        thetaSecond = (3*(thetaFinalPosition - thetaInitPosition)/Math.pow(segmentDuration,2) - ((2*thetaInitVelocity) + thetaFinalVelocity)/segmentDuration);
        thetaThird = ((thetaFinalVelocity+thetaInitVelocity)/Math.pow(segmentDuration,2)) + ((2*(thetaInitPosition-thetaFinalPosition))/Math.pow(segmentDuration,3));


    }

    public void movePath(){
        if (PVT_index <= 0){
            throw new RuntimeException("table is empty :(");
        }
        currentIndex = 1;
        curveState = CurveState.calculatingCoefficients;


    }

    public void release(){
        if (gripperMode== "external"){
            gripServo.setPosition(0.7);
        } else {
            clawServo.setPosition(0.5);
            gripServo.setPosition(0.9);
        }

    }

    public void grip(){
        if (gripperMode== "external"){
            gripServo.setPosition(0.9);
        } else {
            gripServo.setPosition(0.7);
        }

    }



    public boolean isCloseToTarget(double targetBotx, double targetBoty, double botx, double boty, double tolerance) {
        // Calculate the absolute distance between the current position and the target position
        double xDistance = Math.abs(botx - targetBotx);  // Difference in botx
        double yDistance = Math.abs(boty - targetBoty);  // Difference in boty


        // Check if both x and y distances are within the tolerance range
        return (xDistance <= tolerance && yDistance <= tolerance);
    }



}











