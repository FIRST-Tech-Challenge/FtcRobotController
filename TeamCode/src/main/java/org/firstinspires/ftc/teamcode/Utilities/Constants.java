package org.firstinspires.ftc.teamcode.Utilities;

public class Constants {

    //Hardware IDs
    public static final String backLeftDriveID = "backLeftDrive";
    public static final String backRightDriveID = "backRightDrive";
    public static final String frontLeftDriveID = "frontLeftDrive";
    public static final String frontRightDriveID = "frontRightDrive";

    public static final String armID = "arm";
    public static final String extenderID = "extender";
    public static final String intakeID = "intake";
    public static final String wristID = "wrist";

    public static final String armLimitSwitchID = "armLimit";
    public static final String extenderLimitSwitchID = "extenderLimit";


    //Motor Power Values
    public static double teleOPErectorPower = 1.0;
    public static double teleOPArmPower = 1.0;
    public static double autoIntakePower = 1.0;
    public static double teleOPWristIncrement = 0.1;

    public static double launcherPower = 1.0;


    //Drive Encoder/Position Values (FrontLeft Motor is Leader)
    public static final double  driveMotorCPR = 28;
    public static final double driveGearRatio = 12;
    public static final double wheelDiameterInches = 75 / 25.4;
    public static final double driveMotorCPI = (driveMotorCPR * driveGearRatio) /
            (wheelDiameterInches * Math.PI);

    //Erector Encoder/Position Values
    public static final double maxErectorPositionInches = 36;
    public static final double erectorMotorCPR = 28;
    public static final double erectorGearRatio = 125;
    public static double erectorMotorRPI = 1.0;
    public static double erectorMotorCPI = (erectorMotorCPR * erectorGearRatio) * erectorMotorRPI;

    public static double erectorShrunkPosition = 0.0;
    public static double erectorHalfwayPosition = 1000.0;
    public static double erectorErectPosition = 2000.0;

    //Extender
    public static final double extenderMaxPositionInches = 36;
    public static final double extenderMotorCPR = 28;
    public static final double extenderGearRatio = 125;
    public static final double spoolRPI = 1.0;
    public static final double extenderMotorCPI = extenderMotorCPR * extenderGearRatio * spoolRPI;
    public static final double extenderZeroingPower = -0.05;


    //Arm Encoder/Position Values
    public static double armMotorCPR = 28;
    public static double armGearRatio = 240;
    public static int armMotorMaxPosition = (int) (120 / (armMotorCPR * armGearRatio));
    public static int armMotorHalfwayPosition = (int) (90 / (armMotorCPR * armGearRatio));

    public static double armZeroingPower = -0.05;

    //Servo Position Values
    public static double wristMinPosition = 0.0;
    public static double wristMaxPosition = 1.0;

    //Drone Subsystem Values
    public static double flywheelVelocity = 1;

    //PID Values
    public static double driveP = 0.1;
    public static double driveI = 0.0;
    public static double driveD = 0.0;

    public static double turnP = 0.1;
    public static double turnI = 0.0;
    public static double turnD = 0.0;

    public static double erectP = 0.1;
    public static double erectI = 0.0;
    public static double erectD = 0.0;

    public static double armP = 0.1;
    public static double armI = 0.0;
    public static double armD = 0.0;

    public static double initialWristPosition = 0.0;
    public static double wristPositionIncrement = 0.01;

    public static String tensorFlowModelFileName = "yeah, this is a file name";
    public static String tensorFlowRedTeamPropLabel = "TeamPropRed";
    public static String tensorFlowBlueTeamPropLabel = "TeamPropBlue";


    public static double spikeLeftCameraPosition = 426;
    public static double spikeRightCameraPosition = 853;
}
