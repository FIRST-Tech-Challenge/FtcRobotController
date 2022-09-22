package org.firstinspires.ftc.teamcode.ultimategoal2020;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.EncoderCalibration;
import org.firstinspires.ftc.teamcode.ebotsenums.EncoderModel;
import org.firstinspires.ftc.teamcode.ebotsenums.EncoderSetup;
import org.firstinspires.ftc.teamcode.ebotsenums.GyroSetting;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotDesign;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotOrientation;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.StartLine;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.Conveyor;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.Crane;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.EbotsManip2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.Gripper;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.Intake2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.Launcher;
import org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020.RingFeeder;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.AutonParameters;
import org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020.EbotsColorSensor;
import org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020.EbotsDigitalTouch;
import org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020.EbotsRev2mDistanceSensor;
import org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020.EbotsRevBlinkinLedDriver;

import java.util.ArrayList;
import java.util.List;

/**
 *   CLASS:     Robot
 *   INTENT:    Robot is the core object that interfaces with the OpMode
 *              Core components include:  Drive System, Manipulator System, Sensor Network
 */

public class EbotsRobot2020 {
    /***************************************************************
     ******    CLASS VARIABLES
     ***************************************************************/
    // Manip Motors
    private Intake2020 intake ;
    private Conveyor conveyor;
    private Launcher launcher;
    private Crane crane;

    private Gripper gripper;
    private RingFeeder ringFeeder;

    // Manip Devices
    private ArrayList<EbotsManip2020> ebotsManips = new ArrayList<>();

    // Sensor arrays
    private ArrayList<DriveWheel> driveWheels;
    private ArrayList<EncoderTracker> encoderTrackers = new ArrayList<>();
    private ArrayList<EbotsColorSensor> ebotsColorSensors = new ArrayList<>();
    private ArrayList<EbotsDigitalTouch> ebotsDigitalTouches = new ArrayList<>();
    private ArrayList<EbotsRev2mDistanceSensor> ebotsRev2mDistanceSensors = new ArrayList<>();

    // LED driver
    private RevBlinkinLedDriver revBlinkinLedDriver;
    private ArrayList<EbotsRevBlinkinLedDriver> ledDrivers = new ArrayList<>();

    // Camera object detection
    // private TFObjectDetector tfod;

    // Field position
    private Pose2020 actualPose2020;       //Current Pose, which consists of Field Position and Heading
    private Pose2020 targetPose2020;        //Intended destination of the robot

    //These are robot limitations which should be set using test opmodes to collect data
    private final double topSpeed = 50.0;            //  in / s
    private final double angularTopSpeedDeg = 276.92;;  //  degrees / s

    private ArrayList<SizeCoordinate> robotSizeCoordinates;

    //These are speed settings which are configurable, perhaps should move to a Enumeration
    private final double spinMaxSignal = Speed.FAST.getTurnSpeed();      //Max allowable translate speed in range [0-1];
    private final double translateMaxSignal = Speed.FAST.getMaxSpeed(); //Max allowable translate signal in range [0-1];
    private final double superSlowMoMinTranslateSignal = 0.2;    //Min allowable Super Slow Mo speed
    private final double superSlowMoMinSpinSignal = 0.1;    //Min allowable Super Slow Mo speed

    private Alliance alliance;

    private DriveCommand driveCommand;
    private PoseError2020 poseError2020;
    //private EncoderSetup encoderSetup;
    private EbotsMotionController ebotsMotionController;


    private List<LynxModule> expansionHubs;         //Array list of all expansion hubs on robot
    private BNO055IMU imu;
    private double initialGyroOffset;  //When the robot starts, the gyro is zero in whichever orientation the robot is facing
    //  So if robot always faces the center, from the red side, the gyro will read 0 when the
    //  robot is actually facing +90°
    //  This captures the rotation required to bring the field coordinates frame in line with the
    //  the robot coordinate system


    private StopWatch ringFeederCycleTimer = new StopWatch();
    private StopWatch gripperCycleTimer = new StopWatch();

    String logTag = "EBOTS";
    boolean debugOn = false;

    /*****************************************************************
     //******    Constants
     //****************************************************************/

    // ************     LAUNCHER     **********************
    final int  HIGH_GOAL = 1347;
    final int  LOW_GOAL = 1250;
    final int  POWER_SHOTS = 1303;

    // ************     RING FEEDER     **********************
    // ring feeder servo should cycle between 2 positions: RECEIVE and FEED
    // time is used to control cycle
    // cycle is triggered using right trigger
    final double RECEIVE = 0.06;
    final double FEED =    0.37;

    // ************     CRANE     **********************
    // Crane positions for wobble goal
    // Dpad_Up - Lift over wall
    // Dpad_left - move to target zone
    // Dpad_down - Grap wobble goal
    // Dpad_right - Go Vertical

    final int  LIFT_OVER_WALL = 115;
    final int  MOVE_WOBBLE_GOAL = 150;
    final int  GRAB_WOBBLE_GOAL = 155;

    final int CRANE_OVER_WALL_HEIGHT = 222;
    final int CRANE_MIN_CRANE_HEIGHT = 334;
    final int CRANE_DRAG_HEIGHT = 290;
    final int CRANE_VERTICAL_HEIGHT = 145;

    int CRANE_ENCODER_OFFSET = 0;


    // ************     GRIPPER     **********************
    // left_trigger - toggle between open and closed position
    final double GRIPPER_OPEN = 0.45;
    final double GRIPPER_CLOSED = 0.25;


    /*****************************************************************
     //******    Enumerations
     //****************************************************************/
    public enum RobotSize2020 {
        xSize(CsysDirection.X, 18.0),
        ySize(CsysDirection.Y, 18.0),
        zSize(CsysDirection.Z, 18.0);

        CsysDirection csysDirection;
        double sizeValue;

        RobotSize2020(CsysDirection csysDirectionIn, double sizeValueIn){
            this.csysDirection = csysDirectionIn;
            this.sizeValue = sizeValueIn;
        }

        public CsysDirection getCsysDirection() {
            return csysDirection;
        }

        public double getSizeValue() {
            return sizeValue;
        }
    }


    /***************************************************************
     ******    CONSTRUCTORS
     ***************************************************************/

    public EbotsRobot2020() {
        this.driveCommand = new DriveCommand();

        //Build the robot physical dimensions
        robotSizeCoordinates = new ArrayList<>();
        for(RobotSize2020 rs: RobotSize2020.values()){
            robotSizeCoordinates.add(new SizeCoordinate(rs.getCsysDirection(), rs.getSizeValue()));
        }

        //Assumes blue alliance if none stated
        this.alliance = Alliance.BLUE;
        //Assumes a default starting position if none specified
        this.actualPose2020 = new Pose2020(Pose2020.PresetPose.INNER_START_LINE, alliance);     //Defaults to INNER and BLUE
        //When no target pose is given, assume Power Shot Launch position
        this.targetPose2020 = new Pose2020(Pose2020.PresetPose.LAUNCH_POWER_SHOT, alliance);
        this.poseError2020 = new PoseError2020(this);

        this.ebotsMotionController = new EbotsMotionController();
//        this.encoderSetup = EncoderSetup.TWO_WHEELS;    //Default value if none provided
    }
    public EbotsRobot2020(HardwareMap hardwareMap) {
        boolean debugOn = true;
        if(debugOn){
            Log.d(logTag, "Instantiating EbotsRobot...");
        }
        this.driveCommand = new DriveCommand();

        //Build the robot physical dimensions
        robotSizeCoordinates = new ArrayList<>();
        for(RobotSize2020 rs: RobotSize2020.values()){
            robotSizeCoordinates.add(new SizeCoordinate(rs.getCsysDirection(), rs.getSizeValue()));
        }

        //Assumes blue alliance if none stated
        this.alliance = Alliance.BLUE;
        //Assumes a default starting position if none specified
        this.actualPose2020 = new Pose2020(Pose2020.PresetPose.INNER_START_LINE, alliance);     //Defaults to INNER and BLUE
        //When no target pose is given, assume Power Shot Launch position
        this.targetPose2020 = new Pose2020(Pose2020.PresetPose.LAUNCH_POWER_SHOT, alliance);
        this.poseError2020 = new PoseError2020(this);

        this.ebotsMotionController = new EbotsMotionController();

        //  Set the cache strategy for expansion hubs (for control loop speed)
        initializeExpansionHubsForBulkRead(hardwareMap);

        //  Initialize the drive wheels and manip devices
        this.initializeStandardDriveWheels(hardwareMap);
        this.initializeEbotsManips(hardwareMap);


    }


    public EbotsRobot2020(Pose2020 actualPose2020){
        this();     //Call the default constructor
        this.actualPose2020 = actualPose2020;     //Set the input pose
        this.poseError2020 = new PoseError2020(this);   //recalculate error
    }

    public EbotsRobot2020(Pose2020 pose2020, Alliance alliance){
        this(pose2020);     //Set the input pose by calling the above constructor with argument (Pose pose)
        this.alliance = alliance;
    }

    public EbotsRobot2020(AutonParameters autonParameters){
        this();     //chain to constructor with arguments (Pose pose, Alliance alliance)
        this.ebotsMotionController = new EbotsMotionController(autonParameters);
//        this.encoderSetup = autonParameters.getEncoderSetup();  //Set the encoderSetup class variable
    }

    public EbotsRobot2020(Pose2020 pose2020, Alliance alliance, AutonParameters autonParameters){
        this(pose2020, alliance);     //chain to constructor with arguments (Pose pose, Alliance alliance)
        this.ebotsMotionController = new EbotsMotionController(autonParameters);
//        this.encoderSetup = autonParameters.getEncoderSetup();  //Set the encoderSetup class variable
    }

    public EbotsRobot2020(Pose2020 actualPose2020, Alliance alliance, AutonParameters autonParameters, HardwareMap hardwareMap){
        this.driveCommand = new DriveCommand();

        //Build the robot physical dimensions
        robotSizeCoordinates = new ArrayList<>();
        for(RobotSize2020 rs: RobotSize2020.values()){
            robotSizeCoordinates.add(new SizeCoordinate(rs.getCsysDirection(), rs.getSizeValue()));
        }

        //Assumes blue alliance if none stated
        this.alliance = alliance;
        this.actualPose2020 = actualPose2020;     //Set the input pose

        //When no target pose is given, assume Power Shot Launch position
        this.targetPose2020 = new Pose2020(Pose2020.PresetPose.LAUNCH_POWER_SHOT, alliance);
        this.poseError2020 = new PoseError2020(this);

        this.ebotsMotionController = new EbotsMotionController(autonParameters);

        //  Initialize the drive wheels and manip devices
        initializeStandardDriveWheels(hardwareMap);
        initializeEbotsManips(hardwareMap);

        // Initialize exp hubs and sensors
        initializeExpansionHubsForBulkRead(hardwareMap);
        //initialize imu if being used by the auton setup
        if(autonParameters.getGyroSetting() != GyroSetting.NONE) {
            this.initializeImu(hardwareMap);
        }
        //initialize color sensors
        this.initializeColorSensors(hardwareMap);
        //initialize digital touch sensors
        this.initializeEbotsDigitalTouches(hardwareMap);
        //initialize LED lights
        this.initializeEbotsRevBlinkinDriver(hardwareMap);

        // prepare encoderTrackers
        this.initializeEncoderTrackers(autonParameters);

        //  Note CALIBRATION_TWO_WHEEL requires the Encoder Setup be changed after initialization
        if(autonParameters == AutonParameters.CALIBRATION_TWO_WHEEL){
            // During robot creation, the encoder setup was set to THREE_WHEELS to instantiate all three encoders
            // This must be switched back to TWO_WHEELS after instantiation so navigation used TWO_WHEEL algorithm
            // Specifically, this affects PoseChange::calculateRobotMovement() and PoseChange::calculateSpinAngle()
            // Note: enum is singleton, so this must be reset next time this routine is run
            autonParameters.setEncoderSetup(EncoderSetup.TWO_WHEELS);
        }

    }

    public EbotsRobot2020(Pose2020.PresetPose presetPose, Alliance alliance){
        this(new Pose2020(presetPose,alliance), alliance);  //chained to constructor with arguments (Pose pose, Alliance alliance)
    }

    public EbotsRobot2020(Pose2020.PresetPose presetPose, Alliance alliance, AutonParameters autonParameters){
        this(new Pose2020(presetPose,alliance), alliance, autonParameters);  //chained to constructor with arguments (Pose, Alliance, AutonParameters)
    }

    /*****************************************************************
     //******    SIMPLE GETTERS AND SETTERS
     //****************************************************************/
    public DriveCommand getDriveCommand(){
        return driveCommand;
    }

    public DriveWheel getDriveWheel(WheelPosition2020 wheelPosition){
        DriveWheel driveWheel = null;
        for(DriveWheel dw: driveWheels){
            if(dw.getWheelPosition()==wheelPosition){
                driveWheel = dw;
                break;
            }
        }
        return driveWheel;
    }

    public ArrayList<EbotsManip2020> getEbotsManips(){
        return ebotsManips;
    }
    public ArrayList<DriveWheel> getDriveWheels(){
        return driveWheels;
    }
    public ArrayList<EncoderTracker> getEncoderTrackers(){return encoderTrackers;}
    public ArrayList<EbotsColorSensor> getEbotsColorSensors(){return this.ebotsColorSensors;}
    public ArrayList<EbotsDigitalTouch> getEbotsDigitalTouches(){return this.ebotsDigitalTouches;}
    public ArrayList<EbotsRev2mDistanceSensor> getEbotsRev2mDistanceSensors(){return this.ebotsRev2mDistanceSensors;}

    //public RevBlinkinLedDriver getRevBlinkinLedDriver(){return this.revBlinkinLedDriver;}
    public ArrayList<EbotsRevBlinkinLedDriver> getLedDrivers(){return this.ledDrivers;}
    // public TFObjectDetector getTfod(){return this.tfod;}

    public Alliance getAlliance(){return this.alliance;}
    public Pose2020 getActualPose(){return this.actualPose2020;}
    public Pose2020 getTargetPose(){return this.targetPose2020;}
    public PoseError2020 getPoseError(){return this.poseError2020;}
    public BNO055IMU getImu(){return this.imu;}
    public EncoderSetup getEncoderSetup() {return this.ebotsMotionController.getAutonParameters().getEncoderSetup();}
    public EbotsMotionController getEbotsMotionController(){return this.ebotsMotionController;}

    public double getTopSpeed(){ return this.topSpeed;}
    public double getAngularTopSpeedDeg(){ return this.angularTopSpeedDeg;}
    public double getAngularTopSpeedRad(){ return Math.toRadians(this.angularTopSpeedDeg);}

    public double getSizeCoordinate(CsysDirection dir){
        double sizeValue = 0;
        if(robotSizeCoordinates != null && dir != null && robotSizeCoordinates.size() > 0){
            sizeValue = SizeCoordinate.getSizeFromCoordinates(dir, robotSizeCoordinates);
        }
        return sizeValue;
    }

    public Launcher getLauncher(){return launcher;}
    public Crane getCrane(){return crane;}
    public Conveyor getConveyor(){return conveyor;}
    public Intake2020 getIntake(){return intake;}

    public Gripper getGripper(){return gripper;}
    public RingFeeder getRingFeeder(){return ringFeeder;}

    public int getCRANE_MIN_CRANE_HEIGHT(){return CRANE_MIN_CRANE_HEIGHT;}
    public int getCRANE_DRAG_HEIGHT(){return CRANE_DRAG_HEIGHT;}
    public StopWatch getRingFeederCycleTimer(){return ringFeederCycleTimer;};

    public double getMotorPower(DcMotorEx motor){
        double returnPower = 0;
        try{
            returnPower = motor.getPower();
        } catch (Exception e) {
            // do nothing
        }
        return returnPower;
    }

    public double getMotorVelocity(DcMotorEx motor){
        double returnVelocity = 0;
        try{
            returnVelocity = motor.getVelocity();
        } catch (Exception e) {
            // do nothing
        }
        return returnVelocity;
    }


    public void setActualPose(Pose2020 pose2020) {
        boolean debugOn = true;
        if(debugOn) {
            Log.d(logTag, "Robot::setActualPose just set to Actual: " + actualPose2020.toString());
            Log.d(logTag, "Robot::setActualPose it was Actual: " + this.actualPose2020.toString());
        }

        this.actualPose2020 = pose2020;
        //Recalculate error after setting target pose
        this.poseError2020 = new PoseError2020(this);
    }

    public void setTargetPose(Pose2020 targetPose2020) {
        boolean debugOn = true;
        if(debugOn) {
            Log.d(logTag, "Robot::setTargetPose just set to Target: " + targetPose2020.toString());
            Log.d(logTag, "Robot::setTargetPose it was Target: " + this.targetPose2020.toString());
            Log.d(logTag, "Robot::Actual: " + this.actualPose2020.toString());
        }
        this.targetPose2020 = targetPose2020;
        //Recalculate error after setting target pose
        this.poseError2020 = new PoseError2020(this);
    }

    public void setDriveCommand(DriveCommand driveCommandIn){
        this.driveCommand = driveCommandIn;
        this.calculateDrivePowers(driveCommandIn);

    }

    public void setDriveCommand(Gamepad gamepad){
//        this.driveCommand = calculateDriveCommandFromGamepad(gamepad);
        DriveCommand driveCommand = new DriveCommand(gamepad);
        this.driveCommand = driveCommand;
        this.calculateDrivePowers(driveCommand);
    }

    public void setAlliance(Alliance allianceIn){
        this.alliance = allianceIn;
    }


    /*****************************************************************
     //******    CALCULATED PROPERTIES
     //****************************************************************/

    public boolean isUsingVirtualEncoders(){
        boolean isUsingVirtualEncoders = false;
        for(EncoderTracker e: encoderTrackers){
            if(e.getIsVirtual() == true){
                isUsingVirtualEncoders = true;
                break;
            }
        }
        return  isUsingVirtualEncoders;
    }

    public boolean isStartPositionCorrect(){
        EbotsColorSensor.TapeColor startLineColor = EbotsColorSensor.TapeColor.BLUE;
        RobotSide startLineSide = RobotSide.RIGHT;

        if (this.alliance == Alliance.RED) {
            startLineColor = EbotsColorSensor.TapeColor.RED;
            startLineSide = RobotSide.LEFT;
        }

        //Update whether the start position has been achieved
        return EbotsColorSensor.isSideOnColor(this.getEbotsColorSensors(), startLineSide, startLineColor);
    }


    /*****************************************************************
     //******    CLASS INSTANCE METHODS
     //****************************************************************/
    public void updateStartPose(StartLine.LinePosition startLinePosition){
        Pose2020 startingPose2020 = calculateStartingPose(startLinePosition);     //robot object exists at this point
        this.setActualPose(startingPose2020);
    }

    private Pose2020 calculateStartingPose(StartLine.LinePosition startLinePosition){
        //Starting poses are handled in an enumeration within Pose

        Pose2020.PresetPose presetPose;
        if(startLinePosition == StartLine.LinePosition.INNER){
            presetPose = Pose2020.PresetPose.INNER_START_LINE;
        } else{
            presetPose = Pose2020.PresetPose.OUTER_START_LINE;
        }

        Pose2020 startingPose2020 = new Pose2020(presetPose, this.getAlliance());
        return startingPose2020;
    }

    public void setInitialGyroOffset(double gyroReading){
        //  This is run right after creating the robot during initialization
        //  This captures the rotation required to bring the field coordinates frame in line with the
        //  the robot coordinate system
        initialGyroOffset = this.actualPose2020.getHeadingDeg() - gyroReading;
    }

    public void initializeImu(HardwareMap hardwareMap){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        this.setInitialGyroOffset(this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    public void initializeStandardDriveWheels(HardwareMap hardwaremap){
        //Initialize the driveWheels array
        driveWheels = new ArrayList<>();

        //Loop through the enumeration in DriveWheel and create a wheel for each position
        for(WheelPosition2020 pos: WheelPosition2020.values()){
            //Create the drive wheel
            DriveWheel driveWheel = new DriveWheel(pos,hardwaremap);
            //Add it to the array
            driveWheels.add(driveWheel);
        }
    }

    @Deprecated
    public void initializeManipMotors(HardwareMap hardwareMap){
        //  Initialize
        //    4 DC motors: intake, conveyer, shooter, crane
        //    2 Servos:  gripper, ringFeeder


        intake = new Intake2020(hardwareMap);

        conveyor = new Conveyor(hardwareMap);

        launcher = new Launcher(hardwareMap);

        crane = new Crane(hardwareMap);

        gripper = new Gripper(hardwareMap);

        ringFeeder = new RingFeeder(hardwareMap);
    }

    public void initializeEbotsManips(HardwareMap hardwareMap){
        conveyor = new Conveyor(hardwareMap);
        ebotsManips.add(conveyor);
        crane = new Crane(hardwareMap);
        ebotsManips.add(crane);
        gripper = new Gripper(hardwareMap);
        ebotsManips.add(gripper);
        intake = new Intake2020(hardwareMap);
        ebotsManips.add(intake);
        launcher = new Launcher(hardwareMap);
        ebotsManips.add(launcher);
        ringFeeder = new RingFeeder(hardwareMap);
        ebotsManips.add(ringFeeder);
    }


    public void initializeEncoderTrackers(RobotDesign robotDesign){
        if(robotDesign==RobotDesign.SEASON_2020){

        }
    }

    public void initializeEncoderTrackers(AutonParameters autonParameters){
        //boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering Robot.initializeEncoderTrackers(AutonParameters)...");

        boolean isVirtual = autonParameters.usesSimulatedEncoders();
        initializeEncoderTrackers(autonParameters.getEncoderSetup(), isVirtual);

        if(debugOn){
            StringBuilder sb = new StringBuilder();
            sb.append("Number of encoders initialized: ");
            sb.append(this.encoderTrackers.size());
            for(EncoderTracker e: this.encoderTrackers){
                sb.append("\n");
                sb.append(e.toString());
            }
            Log.d(logTag, sb.toString());
        }
    }

    public void initializeEncoderTrackers(EncoderSetup encoderSetup, boolean isVirtual){
        //Initializes encoder trackers and maps them to wheelPosition motors
        // First clear out old encoder trackers if they exist
        if(encoderTrackers.size() > 0){
            Log.d(logTag, encoderTrackers.size() + " Encoders found during loading were expunged!");
            encoderTrackers.clear();
        }

        if(isVirtual) {
            initializeVirtualEncoderTrackers();
        } else{
            EncoderModel encoderModel = encoderSetup.getEncoderModel();

            //DcMotorEx motor, RobotOrientation robotOrientation, EncoderModel encoderModel
            //ToDo:  Find a better way to handle mapping of Encoders.  Should be property of Drivewheel?
//            final DcMotorEx forwardEncoderMotor = this.getDriveWheel(WheelPosition.BACK_RIGHT).getWheelMotor();
            final DcMotorEx forwardEncoderMotor = this.getDriveWheel(WheelPosition2020.BACK_LEFT).getWheelMotor();
//            final DcMotorEx lateralEncoderMotor = this.getDriveWheel(WheelPosition.FRONT_RIGHT).getWheelMotor();
            final DcMotorEx lateralEncoderMotor = this.getDriveWheel(WheelPosition2020.FRONT_LEFT).getWheelMotor();
            EncoderTracker e1 = new EncoderTracker(forwardEncoderMotor, RobotOrientation.FORWARD, encoderModel);
            e1.setEncoderCalibration(EncoderCalibration.FORWARD_LEFT);

            //TODO:  SpinBehavior and ClickDirection should be read from RobotDesign
            e1.setClickDirection(EncoderTracker.ClickDirection.STANDARD);
            e1.setSpinBehavior(EncoderTracker.SpinBehavior.DECREASES_WITH_ANGLE);
            //todo:  Find a better way to apply calibration
            //e1.setSpinRadius(7.944);     //Value from calibration
            encoderTrackers.add(e1);
            EncoderTracker e2 = new EncoderTracker(lateralEncoderMotor, RobotOrientation.LATERAL, encoderModel);
            e2.setEncoderCalibration(EncoderCalibration.LATERAL);
            //e2.setSpinRadius(3.888);
            encoderTrackers.add(e2);

            if (encoderSetup == EncoderSetup.THREE_WHEELS) {
                //Create a second forward encoder
                final DcMotorEx forward2EncoderMotor = this.getDriveWheel(WheelPosition2020.FRONT_LEFT).getWheelMotor();
                EncoderTracker thirdEncoder = new EncoderTracker(forward2EncoderMotor, RobotOrientation.FORWARD, encoderModel);
                thirdEncoder.setSpinBehavior(EncoderTracker.SpinBehavior.DECREASES_WITH_ANGLE);
                thirdEncoder.setEncoderCalibration(EncoderCalibration.FORWARD_LEFT);
                //thirdEncoder.setSpinRadius(7.690);
                encoderTrackers.add(thirdEncoder);
            }
        }
    }

    private void initializeVirtualEncoderTrackers() {
        //Initializes virtual encoders
        encoderTrackers.add(new EncoderTracker(true, RobotOrientation.FORWARD));
        encoderTrackers.add(new EncoderTracker(true, RobotOrientation.LATERAL));

        if (this.getEncoderSetup() == EncoderSetup.THREE_WHEELS) {
            //Create a second forward encoder
            EncoderTracker thirdEncoder = new EncoderTracker(true, RobotOrientation.FORWARD);
            thirdEncoder.setSpinBehavior(EncoderTracker.SpinBehavior.DECREASES_WITH_ANGLE);
            encoderTrackers.add(thirdEncoder);
        }
    }

    public void initializeColorSensors(HardwareMap hardwareMap){
        //Create color sensors used on the robot

        //Make sure the list is empty before initializing
        if(ebotsColorSensors.size() > 0) ebotsColorSensors.clear();

        //Add four color sensors are located at each wheel location, based on SensorLocation enum
        for(EbotsColorSensor.SensorLocation loc: EbotsColorSensor.SensorLocation.values()){
            ebotsColorSensors.add(new EbotsColorSensor(loc, hardwareMap));
        }
    }

    public void initializeEbotsDigitalTouches(HardwareMap hardwareMap){
        //Create digitalTouch sensors used on the robot

        //Make sure the list is empty before initializing
        if(ebotsDigitalTouches.size() > 0) ebotsColorSensors.clear();

        //Add EbotsDigitalTouch sensor for each ButtonFunction enum value
        for(EbotsDigitalTouch.ButtonFunction buttonFunction: EbotsDigitalTouch.ButtonFunction.values()){
            ebotsDigitalTouches.add(new EbotsDigitalTouch(buttonFunction,hardwareMap));
        }
    }

    public void initializeRevBlinkinLedDriver(HardwareMap hardwareMap){
        //Initialize the LED lights
        revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

    }

    public void initializeEbotsRev2mDistanceSensors(HardwareMap hardwareMap){
        for(RobotSide rs: RobotSide.values()){
            ebotsRev2mDistanceSensors.add(new EbotsRev2mDistanceSensor(rs, hardwareMap));
        }
    }

    public void initializeEbotsRevBlinkinDriver(HardwareMap hardwareMap){
        if(debugOn) Log.d(logTag, "Entering Robot::initializeEbotsRevBlinkinDriver...");
        for(EbotsRevBlinkinLedDriver.LedLocation ledLocation: EbotsRevBlinkinLedDriver.LedLocation.values()){
            if(debugOn) Log.d(logTag, "Entering forLoop, ledLocation: " + ledLocation.toString());

            ledDrivers.add(new EbotsRevBlinkinLedDriver(ledLocation, this.alliance, hardwareMap));
        }
        if(debugOn) Log.d(logTag, "Exiting Robot::initializeEbotsRevBlinkinDriver...");

    }

    public void initializeExpansionHubsForBulkRead(HardwareMap hardwareMap) {
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering initializeExpansionHubsForBulkRead...");


        // Generate a list of all the robot's Expansion/Control Hubs
        expansionHubs = hardwareMap.getAll(LynxModule.class);

        // Set all Expansion hubs to use the AUTO Bulk Caching mode.
        // This setting performs one bulk read and allows each sensor to queried once
        // If a second query occurs, then additional bulk reads will occur
        Log.d(logTag, "Robot::initializeExpansionHubsForBulkRead - About to set bulk caching mode...");
        for (LynxModule module : expansionHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            Log.d(logTag, module.getDeviceName() + " caching mode: " + module.getBulkCachingMode());
        }
    }

    public void zeroEncoders(){
        for(EncoderTracker e: encoderTrackers){
            e.zeroEncoder();
        }
    }

    public void bulkReadSensorInputs(long loopDuration, boolean includeColorSensors, boolean includeDistanceSensors){
        //This should be done once per control loop
        //It interfaces with the REV Expansion hubs to read all the values stored in its cache
        //These must be moved to variables for further accessing.
        //Duplicating calls to the hardware will cause additional bulk reads if in AUTO mode, slowing control loop
        //Look in examples ConceptMotorBulkRead for further guidance
        //boolean debugOn = true;
        //boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering Robot.bulkReadSensorInputs...");

        //if using virtual encoders, simulate the loop output
        if(this.isUsingVirtualEncoders()){
            for(EncoderTracker e: this.encoderTrackers){
                if(debugOn) Log.d(logTag, "Sending to simulateLoopOutput: " + e.toString());
                e.simulateLoopOutput(this, loopDuration);
                if(debugOn){
                    StringBuilder sb = new StringBuilder();
                    sb.append("Back in Robot.BulkReadSensorInputs\n");
                    sb.append(e.toString());
                    Log.d(logTag, sb.toString());
                }
            }
        }else {
            //Read in the Encoder Values
            for (EncoderTracker e : encoderTrackers) {
                e.setNewReading();
            }
        }

        //TODO:  take away the EncoderSetup.COMPETITION_BOT condition once the RobotDesign is incorporated into initialization
        boolean readImu = (this.getEncoderSetup() == EncoderSetup.TWO_WHEELS | this.getEncoderSetup() == EncoderSetup.COMPETITION_BOT);
        if(debugOn) Log.d(logTag, "robot::bulkReadSensorInputs readIMU: " + readImu);
        //If the Imu is being used, read it
        if(readImu) {
            //Set the newHeadingReadingDeg variable for the pose
            if(!isUsingVirtualEncoders()){
                // Use the imu if not using virtual encoders
                float gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if(debugOn) Log.d(logTag, "Robot::bulkReadSensorInputs Reading from Gyron: " + String.format("%.2f", gyroReading));
                this.setNewHeadingReadingDegFromGyro(gyroReading);

            } else{
                // Set newHeadingReadingDeg based on best estimate
                double estimatedNewHeading = this.actualPose2020.getHeadingDeg() + estimateHeadingChangeDeg(loopDuration);
                this.actualPose2020.setNewHeadingReadingDeg(estimatedNewHeading);
            }
        }

        //Read the digitalTouches
        for(EbotsDigitalTouch edt: ebotsDigitalTouches){
            edt.setIsPressed();
        }

        //Read the colorSensors
        if(includeColorSensors) {
            for (EbotsColorSensor ecs : ebotsColorSensors) {
                ecs.performHardwareRead();
            }
        }

        if(includeDistanceSensors) {
            //Read the distance Sensors
            for (EbotsRev2mDistanceSensor ds : ebotsRev2mDistanceSensors) {
                ds.setDistanceInches();
            }
        }
    }

    public void bulkReadSensorInputs(long loopDuration, int loopCount, StopWatch stopWatch){
        //OVERLOAD FOR PERFORMANCE TUNING

        //This should be done once per control loop
        //It interfaces with the REV Expansion hubs to read all the values stored in its cache
        //These must be moved to variables for further accessing.
        //Duplicating calls to the hardware will cause additional bulk reads if in AUTO mode, slowing control loop
        //Look in examples ConceptMotorBulkRead for further guidance
        //boolean debugOn = true;
        if(debugOn) Log.d(logTag, "Entering Robot.bulkReadSensorInputs...");

        //Debug parameters
        long splitTimeMillis = stopWatch.getElapsedTimeMillis();
        String operation = "";

        if (debugOn)operation = "ReadEncoders";
        //if using virtual encoders, simulate the loop output
        if(this.isUsingVirtualEncoders()){
            for(EncoderTracker e: this.encoderTrackers){
                //EncoderTracker e = this.encoderTrackers.get(i);
                if(debugOn) Log.d(logTag, "Sending to simulateLoopOutput: " + e.toString());
                e.simulateLoopOutput(this, loopDuration);
                if(debugOn){
                    StringBuilder sb = new StringBuilder();
                    sb.append("Back in Robot.BulkReadSensorInputs\n");
                    sb.append(e.toString());
                    Log.d(logTag, sb.toString());
                }
            }
        }else {
            //Read in the Encoder Values
            for (EncoderTracker e : encoderTrackers) {
                e.setNewReading();
            }
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);



        if (debugOn) operation = "Read IMU";
        boolean readImu = (this.getEncoderSetup() == EncoderSetup.TWO_WHEELS);
        //If the Imu is being used, read it
        if(readImu) {
            //Set the newHeadingReadingDeg variable for the pose
            if(!isUsingVirtualEncoders()){
                // Use the imu if not using virtual encoders
                float gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                this.setNewHeadingReadingDegFromGyro(gyroReading);
            } else{
                // Set newHeadingReadingDeg based on best estimate
                double estimatedNewHeading = this.actualPose2020.getHeadingDeg() + estimateHeadingChangeDeg(loopDuration);
                this.actualPose2020.setNewHeadingReadingDeg(estimatedNewHeading);
            }
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);



        //Read the colorSensors
        if(debugOn) operation = "Read colorSensors";
        for(EbotsColorSensor ecs: ebotsColorSensors){
            ecs.performHardwareRead();
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);


        if(debugOn) operation = "Read digitalTouches";
        //Read the digitalTouches
        for(EbotsDigitalTouch edt: ebotsDigitalTouches){
            edt.setIsPressed();
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);


        //Read the distance Sensors
        if(debugOn) operation = "Read distanceSensors";
        for(EbotsRev2mDistanceSensor ds: ebotsRev2mDistanceSensors){
            ds.setDistanceInches();
        }
        if(debugOn) splitTimeMillis = stopWatch.logSplitTime(logTag, operation, splitTimeMillis, loopCount);

    }


    public void logEncoderTrackers(){
        //boolean debugOn = false;
        StringBuilder sb = new StringBuilder();
        sb.append("Entering Robot.logEncoderTrackers...");
        for(EncoderTracker e: this.encoderTrackers){
            sb.append("\n");
            sb.append(e.toString());
        }
        if(debugOn){
            Log.d(logTag,sb.toString());
        }
    }

    public void testMotors(LinearOpMode opMode, Gamepad gamepad1){
        StopWatch stopWatch = new StopWatch();
        long lockoutTimeMillis = 750L;

        DriveCommand driveCommand = new DriveCommand();
        driveCommand.setMagnitude(0.2);
        this.driveCommand = driveCommand;
        this.calculateDrivePowers(driveCommand);

        for (DriveWheel dw: driveWheels){
            this.stop();
            dw.setCalculatedPower(driveCommand);
            dw.setMotorPower();
            while(!opMode.isStopRequested()
                    && !opMode.isStarted()
                    && !(gamepad1.x && stopWatch.getElapsedTimeMillis()>lockoutTimeMillis)
            ) {
                opMode.telemetry.clearAll();
                opMode.telemetry.addData("Current Motor", dw.getWheelPosition().toString());
                opMode.telemetry.addLine("Push X to go to next motor");
                opMode.telemetry.update();
            }
            stopWatch.reset();
        }

        this.stop();
        this.driveCommand.setMagnitude(0);

    }

    public double estimateHeadingChangeDeg(long timeStepMillis){
        //Returns the estimated change in the robots heading based on the driveCommand and speed attributes of the robot
        //This is used when simulated loop output and the gyro is not available
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering estimateHeadingChangeDeg...");

        double spinSignal = this.driveCommand.getSpin();
        double angularTopSpeedDeg = this.angularTopSpeedDeg;
        double actualAngularSpeedDeg = spinSignal * angularTopSpeedDeg;

        double rotationAngleDeg = actualAngularSpeedDeg * (timeStepMillis / 1000.0);
        return rotationAngleDeg;
    }

    @Deprecated
    public DriveCommand calculateDriveCommandFromGamepad(Gamepad gamepad){
        //  Deprecated, use new DriveCommand(gamepad) instead

        //  Robot Drive Angle is interpreted as follows:
        //
        //      0 degrees -- forward - (Positive X-Direction)
        //      90 degrees -- left   - (Positive Y-Direction)
        //      180 degrees -- backwards (Negative X-Direction)
        //      -90 degrees -- right    (Negative Y-Direction)
        //
        //  NOTE: This convention follows the right hand rule method, where :
        //      +X --> Forward, +Y is Left, +Z is up
        //   +Spin --> Counter clockwise
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag, "Entering calculateDriveCommandFromGamepad...");


        //Read in the gamepad inputs
        double forwardInput = -gamepad.left_stick_y;  //reversing sign because up on gamepad is negative
        double lateralInput = -gamepad.left_stick_x;  //reversing sign because right on gamepad is positive
        double spinInput = -gamepad.right_stick_x;    //Positive means to spin to the left (counterclockwise (CCW) when looking down on robot)

        //todo:  Add the super Slow Mo controls
        //Get the input for Super Slo-Mo
//        double superSloMoInput = 1-gamepad.left_trigger;
//        if(superSloMoInput < superSlowMoMinSignal){
//            superSloMoInput = superSlowMoMinSignal;
//        }
//        translateMaxSignal = superSloMoInput;

        //Set the values for the robot's driveCommand object
        DriveCommand driveCommand = new DriveCommand();
        driveCommand.setMagnitudeAndDriveAngle(forwardInput, lateralInput, this.translateMaxSignal);
        driveCommand.setSpinDrive(spinInput, this.spinMaxSignal);
        return driveCommand;
    }


    @Deprecated
    public void handleManipInput(Gamepad gamepad){
        // handles gamepad input from the controller and actuates motors
        // controls intake, conveyor, launcher, crane, ringFeeder, gripper

        double inputThreshold = 0.3;

        // ************     LAUNCHER   **********************
        // Set the speed for the shooter
        //  Y - HIGH GOAL
        //  B - POWER SHOTS
        //  A - LOW GOAL
        //  X - STOP
        if(gamepad.y){
            launcher.setVelocity(HIGH_GOAL);
//            launcher.setPower(HIGH_GOAL);
        }else if(gamepad.b){
            launcher.setVelocity(POWER_SHOTS);
//            launcher.setPower(POWER_SHOTS);
        }else if(gamepad.a){
            launcher.setVelocity(LOW_GOAL);
//            launcher.setPower(LOW_GOAL);
        }else if(gamepad.x){
            launcher.stop();
        }

        // ************     RING FEEDER     **********************
        // ring feeder servo should cycle between 2 positions: RECEIVE and FEED
        // time is used to control cycle
        // cycle is triggered using right trigger
        final long CYCLE_TIME = 500;    // intended to be time to move between positions

        boolean triggerPressed = Math.abs(gamepad.right_trigger) > inputThreshold;
        //  readyToReceiveRing makes sure that the servo is back to the original position before cycling again
        //  also, conveyors shouldn't feed into shooter if not ready for feed
        boolean cycleTimeout = (ringFeederCycleTimer.getElapsedTimeMillis() > (2*CYCLE_TIME));
        double errorFromReceivePosition = ringFeeder.getPosition() - RECEIVE;
        double errorFromFeedPosition = ringFeeder.getPosition() - FEED;

        //TODO:  figure out why these position error booleans aren't working
        boolean readyToReceiveRing = (Math.abs(errorFromReceivePosition) < 0.02);
        boolean feedPositionReached = (Math.abs(errorFromFeedPosition) < 0.01);

        // check to see if the trigger is pushed and either a)servo returned to receive position or b)cycle timeout
//        if(triggerPressed  && (readyToReceiveRing | cycleTimeout)){
        if(triggerPressed  && (cycleTimeout)){
            ringFeederCycleTimer.reset();
            ringFeeder.setPosition(FEED);
        } else if(ringFeederCycleTimer.getElapsedTimeMillis() > CYCLE_TIME){
            ringFeeder.setPosition(RECEIVE);
        }



        // ************     CRANE     **********************
        // Crane positions for wobble goal
        // Dpad_Up - Lift over wall
        // Dpad_left - move to target zone
        // Dpad_down - Grap wobble goal
        // Dpad_right - stop motor

//        if(gamepad.dpad_up){
//            crane.setTargetPosition(LIFT_OVER_WALL);
//            crane.setPower(1);
//        } else if(gamepad.dpad_left){
//            crane.setTargetPosition(MOVE_WOBBLE_GOAL);
//            crane.setPower(1);
//        } else if(gamepad.dpad_down){
//            crane.setTargetPosition(GRAB_WOBBLE_GOAL);
//            crane.setPower(1);
//        } else if(gamepad.dpad_right){
//            crane.setPower(0);
//        }
        //Crane starts from 0 when folded down to 160 as maximum down position


        int cranePos = crane.getCurrentPosition() + CRANE_ENCODER_OFFSET;
        boolean dragWobble = false;
        boolean liftOverWall = false;
        boolean goVertical = false;

        // get the controller input for crane
        // set boolean values
        double craneInput = 0;
        if(gamepad.dpad_up) {
            craneInput = 1;
            liftOverWall = true;
        } else if(gamepad.dpad_down) {
            craneInput = -1;
        } else if(gamepad.dpad_right){
            craneInput = 1;
            goVertical=true;
        } else if (gamepad.dpad_left){
            craneInput = 1;
            dragWobble = true;
        }

        // Set the max allowable height
        int MAX_HEIGHT = CRANE_VERTICAL_HEIGHT;
        if(dragWobble){
            MAX_HEIGHT = CRANE_DRAG_HEIGHT;
        } else if(liftOverWall){
            MAX_HEIGHT = CRANE_OVER_WALL_HEIGHT;
        } else if(goVertical){
            MAX_HEIGHT = CRANE_VERTICAL_HEIGHT;
        }

        boolean allowUpwardsTravel = cranePos > MAX_HEIGHT;        //only allow upwards travel if greater than max height
        boolean requestingUpwardsTravel = (Math.signum(craneInput) == 1);
        double passPower = 0;
        //  UPWARD Travel

        if (craneInput==0){
            passPower = 0;
        }else if(requestingUpwardsTravel && allowUpwardsTravel) {
            if (dragWobble && cranePos < (MAX_HEIGHT + 5)) {
                passPower = -0.5;
            }else if (liftOverWall && cranePos < (MAX_HEIGHT + 5)) {
                passPower = -0.8;
            }else if(goVertical && cranePos < (MAX_HEIGHT + 5)){
                passPower = -0.8;
            }else {
                passPower = -0.8;
            }
        }
        // if requesting downward travel, and want to go slow at end
        else if(!requestingUpwardsTravel){
            boolean allowDownwardTravel = cranePos < CRANE_MIN_CRANE_HEIGHT;

            if (!allowUpwardsTravel) passPower = 0.8;  //Apply high power while unfolding
            else if (!allowDownwardTravel)
                passPower = 0;        //No power after encoder hits 160;
            else passPower = 0.2;       //Lower power if close to bottom
        }

        crane.setPower(passPower);
//        String f = "%.2f";
//        Log.d(logTag, "---------------------------------");
//        Log.d(logTag, "dragWobble / liftOverWall: " + dragWobble + " / " + liftOverWall);
//        Log.d(logTag, "Crane Pos: " + cranePos);
//        Log.d(logTag, "Crane passPower: " + String.format(f, passPower));
//        Log.d(logTag, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");


        if(gamepad.left_bumper && gamepad.right_bumper & gamepad.right_stick_button){
            resetCraneEncoder();
            CRANE_ENCODER_OFFSET = CRANE_MIN_CRANE_HEIGHT;
        }

        // ************     GRIPPER     **********************
        // left_trigger - toggle between open and closed position

        final long gripperTimeout = 500L;
        boolean gripperToggled = gamepad.left_trigger > inputThreshold
                && gripperCycleTimer.getElapsedTimeMillis() > gripperTimeout;

        if(gripperToggled){
            toggleGripper();
            gripperCycleTimer.reset();
        }

        // ************     CONVEYOR     & Intake  **********************

        // left_stick_y upwards - Feed ring into shooter
        // left_stick_y down - reverse conveyer

        double conveyorInput = -gamepad.left_stick_y;

        // Condition the input signal to either be -1, 0, or 1
        double conveyorPower = (Math.abs(conveyorInput) < inputThreshold) ? 0 : Math.signum(conveyorInput) * 1;

        // if trying to feed ring in, check that the servo is in the receive position
        //if (conveyorPower == 1 && !readyToReceiveRing && !cycleTimeout) conveyorPower = 0;

        conveyor.setPower(conveyorPower);
        intake.setPower(conveyorPower);

    }

    @Deprecated
    public void toggleGripper(){
        boolean isOpen = Math.abs(gripper.getPosition() - GRIPPER_OPEN) < 0.05;
        if(isOpen){
            gripper.setPosition(GRIPPER_CLOSED);
        } else{
            gripper.setPosition(GRIPPER_OPEN);
        }
    }

    @Deprecated
    public void closeGripper(){
        gripper.closeGripper();
    }

    @Deprecated
    public int unfoldCrane(){
        int cranePos = crane.unfoldCrane();
        return cranePos;
    }

    @Deprecated
    public int moveCraneToDragWobbleGoal() {
        int cranePos = crane.getCurrentPosition();
        int MAX_HEIGHT = CRANE_DRAG_HEIGHT;
        boolean allowUpwardsTravel = cranePos > MAX_HEIGHT;        //only allow upwards travel if greater than max height
        double passPower = 0;
        if (allowUpwardsTravel) passPower = (cranePos < (MAX_HEIGHT + 5)) ? -0.4 : -1.0;
        crane.setPower(passPower);
        return cranePos;
    }

    @Deprecated
    public void moveCraneToLiftOverWall(){
        int cranePos = crane.getCurrentPosition();
        int MAX_HEIGHT = CRANE_OVER_WALL_HEIGHT;
        boolean allowUpwardsTravel = cranePos > MAX_HEIGHT;        //only allow upwards travel if greater than max height
        double passPower = 0;
        if (allowUpwardsTravel) passPower = (cranePos < (MAX_HEIGHT + 5)) ? -0.3 : -1.0;
        crane.setPower(passPower);
    }

    @Deprecated
    public void stopCrane(){
        crane.setPower(0);
    }

    @Deprecated
    public void resetCraneEncoder(){
        crane.resetCraneEncoder();
    }

    @Deprecated
    public void feedRing(){
        final long CYCLE_TIME = 500;    // intended to be time to move between positions
        ringFeederCycleTimer.reset();
        ringFeeder.setPosition(FEED);
        while(ringFeederCycleTimer.getElapsedTimeMillis() < CYCLE_TIME){
            //wait for a cycle
        }
        ringFeeder.setPosition(RECEIVE);
    }

    @Deprecated
    public void startLauncher(){
        launcher.setVelocity(HIGH_GOAL);
    }

    @Deprecated
    public void stopLauncher() {
        launcher.setPower(0);
    }

    @Deprecated
    public void startConveyor(){
        conveyor.setPower(1.0);
    }

    @Deprecated
    public void stopConveyor(){
        conveyor.setPower(0);
    }

    @Deprecated
    public void calculateDrivePowers(){
        //Loop through the drive wheels and set the calculated power

        for(DriveWheel dw: driveWheels){
            dw.setCalculatedPower(driveCommand);     //Considers translation and spin
        }

        //Now condition the calculated drive powers
        //  --If magnitude of any drive is greater than 1, then scale all down
        //    Note:  this scaling is different from scaling due to Speed objects
        //           this makes sures the motors aren't over-driven, which would cause erratic controls
        //           For instance, it is possible to have both translate and spin speeds governed to 0.5
        //           but still drive a motor to its full potential of 1.0

        double maxCalculatedPowerMagnitude = getMaxCalculatedPowerMagnitude();
        double maxAllowedPower = 1.0;

        //Apply a scale factor if maxAllowedPower is exceeded
        if(maxCalculatedPowerMagnitude>maxAllowedPower){
            double scaleFactor = maxAllowedPower/maxCalculatedPowerMagnitude;
            this.applyScaleToCalculatedDrive(scaleFactor);
        }
    }

    public void calculateDrivePowers(DriveCommand driveCommand){
        //Loop through the drive wheels and set the calculated power

        for(DriveWheel dw: driveWheels){
            dw.setCalculatedPower(driveCommand);     //Considers translation and spin
        }

        //Now condition the calculated drive powers
        //  --If magnitude of any drive is greater than 1, then scale all down
        //    Note:  this scaling is different from scaling due to Speed objects
        //           this makes sures the motors aren't over-driven, which would cause erratic controls
        //           For instance, it is possible to have both translate and spin speeds governed to 0.5
        //           but still drive a motor to its full potential of 1.0

        double maxCalculatedPowerMagnitude = getMaxCalculatedPowerMagnitude();
        double maxAllowedPower = 1.0;

        //Apply a scale factor if maxAllowedPower is exceeded
        if(maxCalculatedPowerMagnitude>maxAllowedPower){
            double scaleFactor = maxAllowedPower/maxCalculatedPowerMagnitude;
            this.applyScaleToCalculatedDrive(scaleFactor);
        }
    }


    private double getMaxCalculatedPowerMagnitude(){
        //Loop through the drive motors and return the max abs value of the calculated drive
        double maxCalculatedPowerMagnitude=0;
        for(DriveWheel dw: driveWheels){
            //get the absolute power from the current drive motor in the loop
            double curMagnitude = Math.abs(dw.getCalculatedPower());  //absolute value
            if(curMagnitude > maxCalculatedPowerMagnitude){
                maxCalculatedPowerMagnitude = curMagnitude;
            }
        }
        return maxCalculatedPowerMagnitude;
    }

    public void applyScaleToCalculatedDrive(double scaleFactor){
        //Loop through each driveWheel and scale calculatedDrive
        for(DriveWheel dw: driveWheels){
            dw.scaleCalculatedPower(scaleFactor);
        }
        //Also update the DriveCommand since it must be scaled back for virtual encoders
        driveCommand.setMagnitude(driveCommand.getMagnitude() * scaleFactor);
        driveCommand.setSpin(driveCommand.getSpin() * scaleFactor);
    }

    public void drive(){
        //boolean debugOn = false;
        if(debugOn) {
            Log.d(logTag, "Entering robot.drive()...");
            Log.d(logTag, "with " + this.driveCommand.toString());
        }
        //Set the calculatedDrive values to the motors
        for(DriveWheel dw: driveWheels){
            dw.setMotorPower();
        }
    }

    public void stop(){
        //Set the calculatedDrive values to the motors
        for(DriveWheel dw: driveWheels){
            dw.stopMotor();
        }
    }

    public void updateActualPose(){
        //Intended to accept a PoseChange object and update the robot's pose accordingly

        boolean debugOn = false;
        if(debugOn) {
            Log.d(logTag,"Entering Robot:updateActualPose...");
            Log.d(logTag, "Actual: " + actualPose2020.toString());
        }
        // Calculate move since last loop
        PoseChange poseChange = new PoseChange(this);

        //First update heading
        actualPose2020.setHeadingDeg(poseChange.getSpinAngleDeg() + poseChange.getSpinAngleDeg());
        actualPose2020.setX(actualPose2020.getX() + poseChange.getIncrementalFieldMovement().getxPosition());
        actualPose2020.setY(actualPose2020.getY() + poseChange.getIncrementalFieldMovement().getyPosition());
        if(debugOn) {
            Log.d(logTag,"Exiting Robot:updateActualPose...");
            Log.d(logTag, "Incremental Field Movement (x, y): (" +
                    String.format("%.2f", poseChange.getIncrementalFieldMovement().getxPosition()) +
                    ", " +String.format("%.2f", poseChange.getIncrementalFieldMovement().getyPosition()) +
                    ")");
            Log.d(logTag, "Actual: " + actualPose2020.toString());
        }

    }


    public void updateAllSensorValues(){
        // During auton, changes in robot position are calculated based on comparing new readings to sensor values
        // After these calculations have been performed, all new readings must be transferred to the sensor values
        // This method goes through each sensor type and performs the value transfer

        //boolean debugOn = false;
        if(debugOn) Log.d(logTag,"Entering updateAllSensorValues...");


        //Transfer encoder readings to current clicks
        for(EncoderTracker e:encoderTrackers){
            e.updateEncoderCurrentClicks();
        }

        //Transfer newHeadingReadingDeg to headingDeg for actual pose
        this.getActualPose().updateHeadingWithReading();
    }

    public void toggleAlliance(){
        if (alliance == Alliance.RED) {
            alliance = Alliance.BLUE;
        } else {
            alliance = Alliance.RED;
        }
    }

    public void setHeadingFromGyro(double gyroHeading){
        double fieldHeading = calculateHeadingFromGyro(gyroHeading);
        this.actualPose2020.setHeadingDeg(fieldHeading);
    }

    public void setNewHeadingReadingDegFromGyro(double gyroHeading){
        double fieldHeading = calculateHeadingFromGyro(gyroHeading);
        this.actualPose2020.setNewHeadingReadingDeg(fieldHeading);
    }

    private double calculateHeadingFromGyro(double gyroHeading){
        return gyroHeading + this.initialGyroOffset;
    }

    public void logSensorData(String logTag){
        Log.d(logTag, "Actual " + this.actualPose2020.toString());
        Log.d(logTag, EbotsRev2mDistanceSensor.printAll(this.getEbotsRev2mDistanceSensors()));
        Log.d(logTag, EbotsColorSensor.printColorsObserved(this.getEbotsColorSensors()));
        Log.d(logTag, EncoderTracker.printAll(this.getEncoderTrackers()));

        for(EbotsColorSensor ecs: ebotsColorSensors) {
            Log.d(logTag, ecs.toString());
        }
    }

    @Override
    public String toString(){
        //boolean debugOn = false;
        if(debugOn) Log.d(logTag,"Entering robot.ToString...");

        StringBuilder outString = new StringBuilder();
        if(actualPose2020 != null) outString.append("Actual " + actualPose2020.toString());
        if(targetPose2020 != null) outString.append("\n" + "Target " + targetPose2020.toString());
        return outString.toString();
    }}
