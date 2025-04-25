package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * goBilda Pinpoint Odometry Computer offset callibration. Run this OpMode to 
 * automatically determine the offsets from the Pinpoint to the Y and X odometry
 * pods so that the center of the robot is the reference point.  This allows the
 * <X,Y> position to remain constant as we navigate if we rotate to a new angle.
 */
@TeleOp(name = "Pinpoint Odo Offsets", group = "Test")
//@Disabled
public class TestPinpointOffsets extends LinearOpMode {

    //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
    GoBildaPinpointDriver odom = null;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    DcMotorEx frontLeftMotor  = null;
    DcMotorEx frontRightMotor = null;
    DcMotorEx rearLeftMotor   = null;
    DcMotorEx rearRightMotor  = null;

    double  currXinches=0.0, currYinches=0.0, currAngle=0.0;
    double  prevXinches=0.0, prevYinches=0.0, prevAngle=0.0;
    double  minXinches=0.0,  maxXinches=0.0,  minYinches=0.0, maxYinches=0.0;
    double  startXradius = 0.0, startYradius = 0.0, startErrorRadius = 0.0; // mm
    double  currXradius  = 0.0, currYradius  = 0.0, currErrorRadius = 0.0;// mm
    double  startXoffset = 0.0, startYoffset = 0.0; // mm

    // Once the program has determined x/y offsets, those values can be saved in
    // the hardware initialization routine, and this flag can be togged to TRUE
    // to make the program skip the initial setup and go straight to the fine-tuning
    boolean fineTuneOnly = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot hardware
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();
        initializeHardware();

        // Display program information
        if( !fineTuneOnly ) {
            telemetry.addLine("This app confirms the Pinpoint Ypod/Xpod directions");
            telemetry.addLine("(FWD/REV) by moving forward and right, and then");
            telemetry.addLine("rotates the robot to determine the offsets");
        } else {
            telemetry.addLine("This app refines the current Pinpoint Ypod/Xpod");
            telemetry.addLine("offsets provided by repeatedly rotating the ");
            telemetry.addLine("robot and watching <X,Y> movement.");
        }
        telemetry.addData("State", "Ready");
        telemetry.update();
        waitForStart();

        // Do we need to verify Pinpoint setup?
        if( opModeIsActive() && !fineTuneOnly ) {
            verifyEncoderDirectionY();
        }
        if( opModeIsActive() && !fineTuneOnly ) {
            verifyEncoderDirectionX();
        }

        // What is our starting error?
        if( opModeIsActive() ) {
            findStartError();
        }

        // Do we need POSITIVE or NEGATIVE offsets for the X and Y pods?
        if( opModeIsActive() && !fineTuneOnly ) {
            findStartOffsets();
        }

        // Repeatedly refine until  user says to stop
        while( opModeIsActive() ) {
           boolean weAreCloseEnough = fineTuneOffsets();
           if( weAreCloseEnough ) break;
        }

        // Ensure all the motors are stopped when we exit
        driveTrainMotorsZero();
    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void initializeHardware() {
 
        // Locate the odometry controller in our hardware settings
        odom = hardwareMap.get(GoBildaPinpointDriver.class,"odom");   // Control Hub I2C port 3
        if( fineTuneOnly ) { // assume we've run this before and only need to fine-tune
           odom.setOffsets(-148.0, +88.4, DistanceUnit.MM);  // odometry pod x,y offsets relative center of robot
           startXoffset = -148.0; // mm
           startYoffset =  +88.4; // mm
        } else { // assume we're starting with complete unknowns
           odom.setOffsets( 0.0, 0.0, DistanceUnit.MM);      // odometry pod x,y offsets relative center of robot
           startXoffset = 0.0; // mm
           startYoffset = 0.0; // mm
        }
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

        // Set all drivetrain motors to run with encoders.
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        

    } // initializeHardware

    /*--------------------------------------------------------------------------------------------*/
    public void changePinpointOffsets( double xOffset, double yOffset )
    {
        // Instruct Pinpoint to use different offsets
        odom.setOffsets( xOffset, yOffset );
        // Start over using the new offfsets 
        odom.resetPosAndIMU();  // takes 0.25 sec
        // Ensure we don't start taking readings until the reset is complete
        sleep(300);
    } // changePinpointOffsets

    /*--------------------------------------------------------------------------------------------*/
    public void verifyEncoderDirectionY()
    {
        // Note our starting position
        readPinpointUpdateVariables();
        prevXinches = currXinches;
        prevYinches = currYinches;
        prevAngle   = currAngle;
        // Shift drivetrain FORWARD at 25% power for 1/2 second
        driveTrainFwdRev( 0.25 );
        sleep(500 );  // 500 msec
        driveTrainMotorsZero();
        // Note our new position
        readPinpointUpdateVariables();
        // Summarize our findings        
        telemetry.addData("Pinpoint Status", odom.getDeviceStatus() );
        // We should have POSITIVE movement in Y
        double movedYinches = currYinches - prevYinches;
        boolean yPassFail = (movedYinches > 0.20)? true:false;
        telemetry.addData("Y movement","%.2f inches (%s)", movedYinches, ((yPassFail)? "pass":"FAIL") );
        if( yPassFail == false ) {
          telemetry.addLine("Need to reverse Y EncoderDirection setting");
        }
        // We should have nearly ZERO movement in X
        double movedXinches = currXinches - prevXinches;
        boolean xPassFail = (movedXinches < 0.20)? true:false;
        telemetry.addData("X movement","%.2f inches (%s)", movedXinches, ((xPassFail)? "pass":"FAIL") );
        if( xPassFail == false ) {
          telemetry.addLine("Unexpected X shift during +Y navigation");
        }
        // We should have nearly ZERO angle change
        double rotatedDegrees = currAngle - prevAngle;
        boolean anglePassFail = (rotatedDegrees < 3.0)? true:false;
        telemetry.addData("Rotation","%.2f degrees (%s)", rotatedDegrees, ((anglePassFail)? "pass":"FAIL") );
        if( anglePassFail == false ) {
          telemetry.addLine("Excessive rotation during +Y navigation");
        }
        waitForUser();
    } // verifyEncoderDirectionY

    /*--------------------------------------------------------------------------------------------*/
    public void verifyEncoderDirectionX()
    {
        // Note our starting position
        readPinpointUpdateVariables();
        prevXinches = currXinches;
        prevYinches = currYinches;
        prevAngle   = currAngle;
        // Shift drivetrain RIGHT at 25% power for 1/2 second
        driveTrainRightLeft( 0.25 );
        sleep(500 );  // 500 msec
        driveTrainMotorsZero();
        // Note our new position
        readPinpointUpdateVariables();
        // Summarize our findings        
        telemetry.addData("Pinpoint Status", odom.getDeviceStatus() );
        // We should have POSITIVE movement in X
        double movedXinches = currXinches - prevXinches;
        boolean xPassFail = (movedXinches > 0.20)? true:false;
        telemetry.addData("X movement","%.2f inches (%s)", movedXinches, ((xPassFail)? "pass":"FAIL") );
        if( xPassFail == false ) {
          telemetry.addLine("Need to reverse X EncoderDirection setting");
        }
        // We should have nearly ZERO movement in Y
        double movedYinches = currYinches - prevYinches;
        boolean yPassFail = (movedYinches < 0.20)? true:false;
        telemetry.addData("Y movement","%.2f inches (%s)", movedYinches, ((yPassFail)? "pass":"FAIL") );
        if( yPassFail == false ) {
          telemetry.addLine("Unexpected Y shift during +X navigation");
        }
        // We should have nearly ZERO angle change
        double rotatedDegrees = currAngle - prevAngle;
        boolean anglePassFail = (rotatedDegrees < 3.0)? true:false;
        telemetry.addData("Rotation","%.2f degrees (%s)", rotatedDegrees, ((anglePassFail)? "pass":"FAIL") );
        if( anglePassFail == false ) {
          telemetry.addLine("Excessive rotation during +X navigation");
        }
        waitForUser();
    } // verifyEncoderDirectionX

    /*--------------------------------------------------------------------------------------------*/
    // Assuming we start with odom.setOffsets(0,0), this code determines the starting error
    // (as in how big a circle the Pinpoint <X,Y> position traces-out as we rotate 360 degrees).
    public void findStartError()
    {
        // Track the min-max movement of <X,Y> s we rotate 360 degrees
        rotateAtLeast360deg();
        // Save that min-max error radius as our starting point
        startXradius = currXradius;
        startYradius = currYradius;
        startErrorRadius = currErrorRadius;
        // Summarize our findings
        telemetry.addData("Pinpoint Status", odom.getDeviceStatus() );
        telemetry.addData("X/Y/Angle","%.1f, %.1f in  (%.1f deg)", currXinches, currYinches, currAngle );
        telemetry.addData("Starting error", "r=%.2f mm (x=%.2f y=%.2f)", startErrorRadius, startXradius, startYradius);
        waitForUser();
    } // findStartError

    /*--------------------------------------------------------------------------------------------*/
    // Assuming we start with odom.setOffsets(0,0), this code determines whether the offsets for the
    // the X & Y odometry pods are positive or negative numbers (ie, which quadrant are we searching?).
    public void findStartOffsets()
    {
        // For the goBilda odometry pods to provide useful input to the Pinpoint computer as the
        // robot navigates around on the field, they need to be mounted AWAY from the center of
        // rotation (otherwise they'd just pivot and not record any movement when the robot rotates.
        // Let's assume they're mounted at least 2"/50mm away from the center such that changing
        // from odom.setOffsets(0,0) to odom.setOffsets(±50,0) will REDUCE the starting error.
        // We'll need to rotate a total of 4 times to determine if X and Y should be POS or NEG.

        telemetry.addData("Pinpoint Status", odom.getDeviceStatus() );
        telemetry.addData("X/Y/Angle","%.1f, %.1f in  (%.1f deg)", currXinches, currYinches, currAngle );
        telemetry.addData("Starting error", "r=%.2f mm (x=%.2f y=%.2f)", startErrorRadius, startXradius, startYradius);

        // X OFFSET: +50mm
        changePinpointOffsets( +50.0, 0.0);  // x=+50mm, y=0
        rotateAtLeast360deg();
        startXoffset = +50.0;
        startYradius = currYradius;
        startXradius = currXradius;
        startErrorRadius = currErrorRadius;
        telemetry.addData("+50/0", "r=%.2f mm (x=%.2f y=%.2f)", currErrorRadius, currXradius, currYradius);
        // X OFFSET: -50mm
        changePinpointOffsets( -50.0, 0.0);  // x=-50mm, y=0
        rotateAtLeast360deg();
        telemetry.addData("-50/0", "r=%.2f mm (x=%.2f y=%.2f)", currErrorRadius, currXradius, currYradius);
        // Is this a better result than the first one?
        // (X pod offset is measured along the Y axis!)
        if( currErrorRadius < startErrorRadius ) {
           startXoffset = -50.0;
           startYradius = currYradius;
           startXradius = currXradius;
           startErrorRadius = currErrorRadius;
        }
        
        // Y OFFSET: +50mm
        changePinpointOffsets( startXoffset, +50.0);  // x=±50mm, y=+50mm
        rotateAtLeast360deg();
        telemetry.addData( ((startXoffset>0)? "+50/+50":"-50/+50:"),"r=%.2f mm (x=%.2f y=%.2f)", currErrorRadius, currXradius, currYradius);
        // Is this a better result than the first one?
        // (Y pod offset is measured along the X axis!)
        if( currErrorRadius < startErrorRadius ) {
            startYoffset = +50.0;
            startYradius = currYradius;
            startXradius = currXradius;
            startErrorRadius = currErrorRadius;
        }
        // Y OFFSET: -50mm
        changePinpointOffsets( startXoffset, -50.0);  // x=±50mm, y=-50mm
        rotateAtLeast360deg();
        telemetry.addData( ((startXoffset>0)? "+50/-50":"-50/-50:"),"r=%.2f mm (x=%.2f y=%.2f)", currErrorRadius, currXradius, currYradius);
        // Is this a better result than the first one?
        if( currErrorRadius < startErrorRadius ) {
            startYoffset = -50.0;
            startYradius = currYradius;
            startXradius = currXradius;
            startErrorRadius = currErrorRadius;
        }

        // Lets report our findings
        telemetry.addData("Best offsets", "x=%.1f mm y=%.1f mm", startXoffset, startYoffset );
        telemetry.addData("Current error", "r=%.2f mm (x=%.2f y=%.2f)", startErrorRadius, startXradius, startYradius);
        waitForUser();
    } // findStartOffsets

    /*--------------------------------------------------------------------------------------------*/
    // This routine assumes that startXoffset & startYoffset are already populated with values
    // in the correct positive/negative domain, and we just need to step them toward values that
    // minimizes the error terms.  Unfortunately, an error in either offset can contribute to the
    // center being off and tracing out a circle, so we take small steps in both X and Y to dial
    // it in to the true center offsets.
    public boolean fineTuneOffsets() {
        double stepX,stepY, posOffset,negOffset, convergeFactor;
        String posYstr= "x/+y", negYstr= "x/-y";

        telemetry.addData("Pinpoint Status", odom.getDeviceStatus());
        telemetry.addData("X/Y/Angle", "%.1f, %.1f in  (%.1f deg)", currXinches, currYinches, currAngle);
        telemetry.addData("Starting offsets", "x=%.2f mm y=%.2f mm", startXoffset, startYoffset);
        telemetry.addData("Starting error", "r=%.2f mm (x=%.2f y=%.2f)", startErrorRadius, startXradius, startYradius);
        // How fast we try to converge the error toward zero depends on how close we are to the center
        convergeFactor = (startErrorRadius > 10.0)? 0.75 : 0.25;
        // Determine the ±step size to use for the X & Y offsets
        // (zero means we're starting from fineTuneOnly, with no prior results)
        stepX = (startYradius == 0.0) ? 2.0 : (convergeFactor * startYradius); // mm
        stepY = (startXradius == 0.0) ? 2.0 : (convergeFactor * startXradius); // mm
        telemetry.addData("Step size", "x=%.1f mm y=%.1f mm", stepX, stepY);
        // Compute the two X offsets we're about to test
        posOffset = startXoffset + stepX;
        negOffset = startXoffset - stepX;
        // X OFFSET: +step
        changePinpointOffsets(posOffset, startYoffset);
        rotateAtLeast360deg();
        telemetry.addData("+x/y", "x=%.2f mm y=%.2f mm", currXradius, currYradius);
        // Is this better than where we started?
        // (X pod offset is measured along the Y axis!)
        if( currErrorRadius < startErrorRadius ) {
            startXoffset = posOffset;
            startYradius = currYradius;
            startXradius = currXradius;
            startErrorRadius = currErrorRadius;
            posYstr = "+x/+y";
            negYstr = "+x/-y";
        }
        // X OFFSET: -step
        changePinpointOffsets( negOffset, startYoffset );
        rotateAtLeast360deg();
        telemetry.addData("-x/y", "x=%.2f mm y=%.2f mm", currXradius, currYradius);
        // Is this even better than our starting/positive test?
        if( currErrorRadius < startErrorRadius ) {
            startXoffset = negOffset;
            startYradius = currYradius;
            startXradius = currXradius;
            startErrorRadius = currErrorRadius;
            posYstr = "-x/+y";
            negYstr = "-x/-y";
        }
        // Compute the two X offsets we're about to test
        posOffset = startYoffset + stepY;
        negOffset = startYoffset - stepY;
        // Y OFFSET: +step
        changePinpointOffsets( startXoffset, posOffset );
        rotateAtLeast360deg();
        telemetry.addData( posYstr,"x=%.2f mm y=%.2f mm", currXradius, currYradius);
        // Is this better than where we started?
        // (Y pod offset is measured along the X axis!)
        if( currErrorRadius < startErrorRadius ) {
            startYoffset = posOffset;
            startXradius = currXradius;
            startYradius = currYradius;
            startErrorRadius = currErrorRadius;
        }
        // Y OFFSET: -step
        changePinpointOffsets( startXoffset, negOffset );
        rotateAtLeast360deg();
        telemetry.addData( negYstr,"x=%.2f mm y=%.2f mm", currXradius, currYradius);
        // Is this even better and our starting/positive test?
        if( currErrorRadius < startErrorRadius ) {
            startYoffset = negOffset;
            startXradius = currXradius;
            startYradius = currYradius;
            startErrorRadius = currErrorRadius;
        }
        // Lets report our findings
        telemetry.addData("Best offsets", "x=%.2f mm y=%.2f mm", startXoffset, startYoffset );
        telemetry.addData("Current error", "r=%.2f mm (x=%.2f y=%.2f)", startErrorRadius, startXradius, startYradius);
        boolean keepGoing = checkWithUser();
        return !keepGoing;
    } // fineTuneOffsets
    
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
        // Assuming rotation only, how much has <X,Y> shifted?
        currXradius = 25.4 * (maxXinches-minXinches)/2.0;  // rotate 180deg; max-min is the diameter of the circle
        currYradius = 25.4 * (maxYinches-minYinches)/2.0;  // of error relative to the true center of the robot
        currErrorRadius = Math.sqrt( (currXradius * currXradius) + (currYradius * currYradius) );
    } // readPinpointUpdateVariables

    /*--------------------------------------------------------------------------------------------*/
    public void rotateAtLeast360deg()
    {
        double turnPower = -0.10;
        // Reset our min-max tracking variables
        minXinches=0.0; maxXinches=0.0;
        minYinches=0.0; maxYinches=0.0;
        // Note our starting position
        readPinpointUpdateVariables();
        prevAngle = currAngle;
        double accumulatedAngle = 0.0;
        // Rotate drivetrain clockwise at 33% power for 360deg.  Start at 10% power to ensure
        // no slippage, and ramp down at the end to lower power because stopping suddenly from
        // 33% power causes a small shift in X/Y position due to momentum that we do not want.
        driveTrainTurn( turnPower );
        while( opModeIsActive() ) {
            sleep(20); // allow some rotation (plus measurement time)
            // How many degrees did we rotate during that period?
            readPinpointUpdateVariables();
            double deltaAngle = Math.max(currAngle,prevAngle) - Math.min(currAngle,prevAngle);
            if( deltaAngle >  180.0 ) { deltaAngle = 360.0 - deltaAngle; }
            // Add that to our running total
            accumulatedAngle += deltaAngle;
            // Are we approaching a full circle?
            if( accumulatedAngle > 356.0) break;  // waiting until 360 results in overshoot
            // Reset for the next iteration
            prevAngle = currAngle;
            // Ramp up at the start (-0.10 to -0.50)
            if( accumulatedAngle < 180.0 ) {
                turnPower = -0.10 - (accumulatedAngle/100.0);  // 1deg=-0.11   40deg=-0.50
                if( turnPower < -0.50 ) turnPower = -0.50;
            } else { // Ramp down at the end (-0.50 to -0.07)
                turnPower = -0.06 - (360.0 - accumulatedAngle)/100.0; // 316deg=-0.50 359deg=-0.07
                if( turnPower < -0.50 ) turnPower = -0.50;
            }
            driveTrainTurn( turnPower );
        }
        driveTrainMotorsZero();
        sleep(500); // ensure fully stopped
        // Note our final position
        readPinpointUpdateVariables();
    } // rotateAtLeast360deg

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

} // TestPinpointOffsets
