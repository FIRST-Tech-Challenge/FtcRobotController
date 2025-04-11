package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * goBilda Pinpoint Odometry Computer offset callibration. Run this OpMode to 
 * automatically determine the offsets from the Pinpoint to the Y and X odometry
 * pods so that the center of the robot is the reference point.  This allows the
 * <X,Y> position to remain constant as we navigate if we rotate to a new angle.
 */
@TeleOp(name = "Drivetrain Min Power", group = "Test")
//@Disabled
public class TestDriveTrainMinPower extends LinearOpMode {

    //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
    GoBildaPinpointDriver odom = null;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    DcMotorEx frontLeftMotor  = null;
    DcMotorEx frontRightMotor = null;
    DcMotorEx rearLeftMotor   = null;
    DcMotorEx rearRightMotor  = null;

    double  currXinches=0.0, currXvel=0.0;
    double  currYinches=0.0, currYvel=0.0;
    double  currAngle=0.0,   currAnglevel=0.0;
    double  prevXinches=0.0, prevYinches=0.0, prevAngle=0.0;
    double  minXinches=0.0, maxXinches=0.0, minYinches=0.0, maxYinches=0.0;

    public boolean          driveLogging = false; // only enable during development!!
    public final static int LOG_SIZE = 256;   // 256 entries = 2.5 seconds @ 10msec/100Hz

    protected double[] logTime = new double[LOG_SIZE];  // Drive time (msec)
    protected double[] logDist = new double[LOG_SIZE];  // Drive distance (mm)

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot hardware
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();
        initializeHardware();

        // Display program information
        telemetry.addLine("This app determines the minimum drivetrain");
        telemetry.addLine("motor power that will actual move the bot.");
        telemetry.addData("State", "Ready");
        telemetry.update();
        waitForStart();

        // Automate the process...
        findMinDrivePower();

        // Ensure all the motors are stopped when we exit
        driveTrainMotorsZero();
    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void initializeHardware() {
 
        // Locate the odometry controller in our hardware settings
        odom = hardwareMap.get(GoBildaPinpointDriver.class,"odom");   // Control Hub I2C port 3
        odom.setOffsets(-148.0, +88.4, DistanceUnit.MM);  // odometry pod x,y offsets relative center of robot
        odom.setEncoderResolution( GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD );
        odom.setEncoderDirections( GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                   GoBildaPinpointDriver.EncoderDirection.REVERSED );
        odom.resetPosAndIMU();

        // Query hardware info
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class,"FrontLeft");  // REVERSE
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"FrontRight"); // forward
        rearLeftMotor   = hardwareMap.get(DcMotorEx.class,"RearLeft");   // REVERSE
        rearRightMotor  = hardwareMap.get(DcMotorEx.class,"RearRight");  // forward

        // Set motor position-power direction
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all drivetrain motors to zero power
        driveTrainMotorsZero();

        // Set all drivetrain motors to run without encoders
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        

    } // initializeHardware

    /*--------------------------------------------------------------------------------------------*/
    public void findMinDrivePower()
    {
        // In order to determine the minimum power that will actually move or rotate the robot,
        // we run without encoders (otherwise the PID control would eventually build up enough
        // power to move, even with a lower power setting).  That's bad, because while it will
        // EVENTUALLY move the robot, that's SLOW so we want to know the true motor power setting
        // that will move the mass of the robot without any buildup.
        readPinpointUpdateVariables();
        telemetry.addData("Pinpoint Status", odom.getDeviceStatus() );
        telemetry.addData("Position","x=%.2f, y=%.2f in  (%.2f deg)", currXinches, currYinches, currAngle );
        telemetry.addData("Velocity","%.2f, %.2f inch/sec  (%.2f deg/sec)", currXvel, currYvel, currAnglevel );


        waitForUser();
    } // findMinDrivePower

    /*--------------------------------------------------------------------------------------------*/
    // Only one answer is allowed here, and when user presses the "X" key we continue
    public void waitForUser()
    {
        // Instruct the Pinpoint computer to update the current position
        telemetry.addLine("Press X to continue...");
        telemetry.update();
        boolean gamepad1_cross_now = gamepad1.cross;
        // Loop/wait until user has viewed  the telemetry and pressed X
        while( opModeIsActive() ){
            // Update the gamepad inputs
            boolean gamepad1_cross_last = gamepad1_cross_now;
            gamepad1_cross_now = gamepad1.cross;
            // Have we just transitioned from NOT-PRESSED to PRESSED?
            if( gamepad1_cross_now && !gamepad1_cross_last) break;
            // Pause, then check again
            sleep(75);
        }
    } // waitForUser

    /*--------------------------------------------------------------------------------------------*/
    // Function returns TRUE to continue/CIRCLE or FALSE to stop/SQUARE
    public boolean checkWithUser()
    {
        // Instruct the Pinpoint computer to update the current position
        telemetry.addLine("Press CIRCLE to refine further or SQUARE to stop...");
        telemetry.update();
        boolean gamepad1_circle_now = gamepad1.circle;
        boolean gamepad1_square_now = gamepad1.square;
        // Loop/wait until user has viewed  the telemetry and pressed a button
        while( opModeIsActive() ) {
            // Update the gamepad inputs
            boolean gamepad1_circle_last = gamepad1_circle_now;
            boolean gamepad1_square_last = gamepad1_square_now;
            gamepad1_circle_now = gamepad1.circle;
            gamepad1_square_now = gamepad1.square;
            // Have we just transitioned from NOT-PRESSED to PRESSED?
            if( gamepad1_circle_now && !gamepad1_circle_last) return true;
            if( gamepad1_square_now && !gamepad1_square_last) return false;
            // Pause, then check again
            sleep(75);
        }
        // should never get here, but in case we do we need a return value
        return false;
    } // checkWithUser

    /*--------------------------------------------------------------------------------------------*/
    public void readPinpointUpdateVariables()
    {
        // Instruct the Pinpoint computer to update the current position
        odom.update();
        Pose2D pos = odom.getPosition();  // x,y pos in inch; heading in degrees
        // Update our local variables for POSITION
        currXinches = pos.getX(DistanceUnit.INCH);
        currYinches = pos.getY(DistanceUnit.INCH);
        currAngle   = pos.getHeading(AngleUnit.DEGREES);
        // Update our tracking min-max variables
        if(currXinches<minXinches){minXinches=currXinches;} if(currXinches>maxXinches){maxXinches=currXinches;}
        if(currYinches<minYinches){minYinches=currYinches;} if(currYinches>maxYinches){maxYinches=currYinches;}
        // Update our local variables for VELOCITY
        Pose2D vel = odom.getVelocity();
        currXvel = vel.getX(DistanceUnit.INCH);            // inch/sec
        currYvel = vel.getY(DistanceUnit.INCH);            // inch/sec
        currAnglevel = vel.getHeading(AngleUnit.DEGREES);  // deg/sec

    } // readPinpointUpdateVariables

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to drive straight FORWARD (Ex: +0.10) or REVERSE (Ex: -0.10)        */
    public void driveTrainFwdRev( double motorPower )
    {
        frontLeftMotor.setPower(  motorPower );
        frontRightMotor.setPower( motorPower );
        rearLeftMotor.setPower(   motorPower );
        rearRightMotor.setPower(  motorPower );
    } // driveTrainFwdRev

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to strafe RIGHT (Ex: +0.10) or LEFT (Ex: -0.10)                     */
    public void driveTrainRightLeft( double motorPower )
    {
        frontLeftMotor.setPower(   motorPower );
        frontRightMotor.setPower( -motorPower );
        rearLeftMotor.setPower(   -motorPower );
        rearRightMotor.setPower(   motorPower );
    } // driveTrainRightLeft

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to turn clockwise (Ex: +0.10) or counterclockwise (Ex: -0.10)       */
    public void driveTrainTurn( double motorPower )
    {
        frontLeftMotor.setPower( -motorPower );
        frontRightMotor.setPower( motorPower );
        rearLeftMotor.setPower(  -motorPower );
        rearRightMotor.setPower(  motorPower );
    } // driveTrainTurn

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotorsZero()
    {
        frontLeftMotor.setPower( 0.0 );
        frontRightMotor.setPower( 0.0 );
        rearLeftMotor.setPower( 0.0 );
        rearRightMotor.setPower( 0.0 );
    } // driveTrainMotorsZero

} // TestDriveTrainMinPower
