package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.teamcode.utilities.Constants2021.FORE_AFT_TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.utilities.Constants2021.INCH_PER_TICK;
import static org.firstinspires.ftc.teamcode.utilities.Constants2021.TICKS_PER_INCH;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CASH_Drive_Library {
    public OpMode _opMode;

    //
    public void init(OpMode opMode){
        _opMode = opMode;
    }

    //Properties to setup motors
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public DcMotor foreAftEncoder;
    public ColorSensor floorColorSensor;
    public IMUUtility imu;

    public DistanceSensor rightFreightDetector;
    public DistanceSensor leftFreightDetector;

    public double FORWARD = -90;
    public double REVERSE = 90;
    public double RIGHT = 180;
    public double LEFT = 0;

    public double TURN_RIGHT = 1;
    public double TURN_LEFT = -1;

    double LF_motorPower = 0;
    double RF_motorPower = 0;
    double LR_motorPower = 0;
    double RR_motorPower = 0;
    double motorCommands[] = new double[]{LF_motorPower, RF_motorPower,LR_motorPower, RR_motorPower};

    //THese parameters are used for the RatelimitCommands method.
    double prev_XCMD = 0;
    double prev_YCMD=0;
    double  prev_TCMD=0;
    double RATELIMIT=.055;  //NOTE:  This value depends on the loop time of the main loop
    double DEADBAND = .025;  //Not used yet

    private boolean IsOpModeActive()
    {
        return ((LinearOpMode)_opMode).opModeIsActive();
    }
    public void Stop() {
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
    }

    public int GetTickCountsFromDistance(double distance, boolean useForAftEncoder){
        if (useForAftEncoder){
            return (int)Math.round(FORE_AFT_TICKS_PER_INCH * distance);
        }
        else {
            return (int)Math.round(TICKS_PER_INCH*distance);
        }

    }
    public int GetTickCountsFromForeAftEncoder(double distance)
    {
        return (int)Math.round(FORE_AFT_TICKS_PER_INCH * distance);
    }
    //Used to show how far we went Fowared/Revers. Usually used in autonomous mode
    public double GetForeAftMovementFromTicks()
    {
        //    return INCH_PER_TICK * leftFrontMotor.getCurrentPosition();
        return (1/FORE_AFT_TICKS_PER_INCH) * foreAftEncoder.getCurrentPosition();
    }
    //Used to show how far we went Left to Right. Usually used in autonomous mode
    public double GetLeftRightMovementFromTicks()
    {
        return INCH_PER_TICK * rightFrontMotor.getCurrentPosition();
    }
    public int getRightRearEncoderTick() {
        return rightRearMotor.getCurrentPosition();
    };
    public int GetTicksCountForMovement(double distance)
    {
        return (int)Math.round(TICKS_PER_INCH * distance);
    }
    public void ResetEncoder()
    {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void EnableEncoders()
    {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetForeAftEncoder()
    {
        foreAftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foreAftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move Robot takes an angle and magnitude and calculates the power needed for each motor.
    // This is based on the information gathered from the following site:
    //https://seamonsters-2605.github.io/archive/mecanum/
    //
    // Ref frame:  Cartisian Coordinate system where the pos Y axis points out the front of the
    // robot and the pos X axis points out the right of the robot.
    // directionStick_x = left Joystick x value
    // directionStick_y = left Joystick y value
    // turnStick_x = right Joystick x value

    //  Angle:  The desired direction the robot should go relative to the pos X axis
    //  Magnitude:  The desired speed you want the robot to go.  1 is max speed.
    public void MoveRobotTeliOp(double directionStick_y,double directionStick_x, double turnStick_x, boolean use_rate_limit, boolean use_exponent_smoothing)
    {
        double YCMD, XCMD, TCMD;
//        XCMD = directionStick_x;
        XCMD = rateLimitCmds(directionStick_x,prev_XCMD,false,false);
//        YCMD = directionStick_y;
        YCMD = rateLimitCmds(directionStick_y,prev_YCMD,false,false);
//        TCMD = turnStick_x; //rateLimitCmds(turnStick_x,prev_TCMD,use_rate_limit,use_exponent_smoothing);
        TCMD = rateLimitCmds(turnStick_x,prev_TCMD,use_rate_limit,false);

//        RobotLog.d(String.format("INT THE XCMD GT RL: XCMD: %.03f prev_XCMD: %.03f Diff :  %.03f ",XCMD,prev_XCMD,prev_XCMD-X_sign*RATELIMIT)));
        prev_XCMD = XCMD;
        prev_YCMD = YCMD;
        prev_TCMD = TCMD;

        double LFMP, RFMP, LRMP, RRMP;

        //Computes angle and magnitude of joystick command from left stick x and y.
        double angle;
        double magnitude;
        double fullSpeedFactor = 1.4144;
        angle = Math.atan2(YCMD, XCMD);
        magnitude = Math.sqrt((XCMD * XCMD) + (YCMD * YCMD))*fullSpeedFactor;
        RobotLog.d(String.format("CASH: X: %.03f Y: %.03f Angle:  %.03f Magnitued: %.03f",directionStick_x,directionStick_y,angle,magnitude));

        motorCommands[0] = (Math.sin(angle + (.25 * 3.14)) * magnitude) - TCMD;//LF_motorPower
        motorCommands[1] = (Math.sin(angle - (.25 * 3.14)) * magnitude) + TCMD;//RF_motorPower
        motorCommands[2] = (Math.sin(angle - (.25 * 3.14)) * magnitude) - TCMD;//LR_motorPower
        motorCommands[3] = (Math.sin(angle + (.25 * 3.14)) * magnitude) + TCMD;//RR_motorPower

        //Need to find max value so we can scale commands because some may be larger than 1.
        //RobotLog.d(String.format("CASH: Motor Commands:  %.03f",motorCommands[0]));
        double maxCommandValue = abs(motorCommands[0]);

        for(int i=1;i < motorCommands.length;i++)
        {
            if(abs(motorCommands[i]) > maxCommandValue)
            {
                maxCommandValue = abs(motorCommands[i]);
            }
//            RobotLog.d(String.format("CASH: Max Command Value:  %.03f",maxCommandValue));
        }

        LFMP = motorCommands[0];
        RFMP = motorCommands[1];
        LRMP = motorCommands[2];
        RRMP = motorCommands[3];
        if (maxCommandValue > 1 )
        {
            LFMP = motorCommands[0]/maxCommandValue;
            RFMP = motorCommands[1]/maxCommandValue;
            LRMP = motorCommands[2]/maxCommandValue;
            RRMP = motorCommands[3]/maxCommandValue;
        }
        leftFrontMotor.setPower(LFMP);
        leftRearMotor.setPower(LRMP);
        rightFrontMotor.setPower(RFMP);
        rightRearMotor.setPower(RRMP);

        RobotLog.d(String.format("CASH: LF: %.03f, RR: %.03f,RF: %.03f,LR: %.03f",LFMP,RRMP,RFMP,LRMP));
    }

    //This moves the robot in autonomous mode with 3 inputs:
    //direction:  angle in degrees you want the robot to translate. 0 degrees is out the right of
    //the robot.  90 degrees is out the front of the robot
    //speed:  0 to 1 where 1 is max speed in the direction above
    public boolean MoveRobotAuto(double direction_deg, double raw_speed,double desired_dist_inch,boolean lookForFreight, OpMode opmode, boolean useForeAftEncoder)
    {
        double LFMP, RFMP, LRMP, RRMP,correction;
        double direction_rad = direction_deg * 3.14/180;
        if (raw_speed > 1)
        {
            raw_speed = 1;
        }
        else if(raw_speed < 0)
        {
            raw_speed = 0;
        }
        double fullSpeedFactor = 1.4144;
        double speed = raw_speed*fullSpeedFactor;

        int desiredTicks = 0;
        if (direction_deg == FORWARD){
            desiredTicks = GetTickCountsFromDistance(desired_dist_inch,false);
//            desiredTicks = GetTickCountsFromForeAftEncoder(desired_dist_inch);
        }else if (direction_deg == REVERSE){
            desiredTicks = GetTickCountsFromDistance(desired_dist_inch,false);
//            desiredTicks = GetTickCountsFromForeAftEncoder(desired_dist_inch);
        }else if (direction_deg == RIGHT){
            desiredTicks = GetTickCountsFromDistance(desired_dist_inch,false);
//            desiredTicks = GetTicksCountForMovement(desired_dist_inch);
        } else if (direction_deg == LEFT){
            desiredTicks = GetTickCountsFromDistance(desired_dist_inch,false);
//            desiredTicks = GetTicksCountForMovement(desired_dist_inch);
        }

        //Must check that op mode is active for any while loop
        boolean achievedDistance = false;
        boolean foundFreight = false;

        //parameters to detect freight
        double distanceToFreight = 10000;
        int countsOfDetection = 0;
        ///
        int iterations = 0;
        while(((LinearOpMode)opmode).opModeIsActive() && (!achievedDistance && !foundFreight))
        {
            //Check if we have reached to desired distance
            int currentPosition;
            if (direction_deg == FORWARD){
                if (useForeAftEncoder){
                    currentPosition=foreAftEncoder.getCurrentPosition();
                }else{
                    currentPosition=rightRearMotor.getCurrentPosition();
                }

            }else if (direction_deg == REVERSE){
                if (useForeAftEncoder){
                    currentPosition=foreAftEncoder.getCurrentPosition();
                }else{
                    currentPosition=rightRearMotor.getCurrentPosition();
                }
            }else if (direction_deg == RIGHT){
                currentPosition=rightRearMotor.getCurrentPosition();
            }else{
                currentPosition=rightRearMotor.getCurrentPosition();
            }

            RobotLog.i(String.format("Direction: %s TicksNeeded: %03d  CurrentTicks: %03d",direction_deg, desiredTicks,currentPosition));
            if(abs(currentPosition) >= abs(desiredTicks)){achievedDistance = true;}


            //Check if we are looking for freight...run this code
            if (lookForFreight)
            {

                double DistToFreight = rightFreightDetector.getDistance(DistanceUnit.MM);
                RobotLog.i(String.format("Looking For Freight.  Distance is: %03f",DistToFreight));
                if (DistToFreight < 48*25.4)
                {
                    countsOfDetection = countsOfDetection +1;
                    RobotLog.i(String.format("Found Freight.  Distance is: %03f and Counts are: %03d",DistToFreight,countsOfDetection));
                    if (countsOfDetection > 4){
                        foundFreight = true;
                        RobotLog.i(String.format("Yeah we found freight!!!!!.  Stopping Robot and returning true"));
                    }
                }

            }


            //This is for rotation correction via IMU.  If the robot starts to turn left or right due
            // to the floor or some obstacle, it will compensate and straighten itself out.  The below
            // method returns the amount of correction needed and with the correct sign.
            correction = 0;
            if (((LinearOpMode)opmode).opModeIsActive())
            {
                correction = imu.checkDirection();
//                correction = 0;
                RobotLog.i(String.format("CORRECTION IS: %f",correction));
            }    else
            {
                this.Stop();
            }



            //This uses the fore aft encoder to compensate only for left and right motion.
            // The encoder can sense if the robot is going forward or reverse while sliding left/right
            // across the floor.  This helps the robot move left and right in a straight line.
            double lateralCorrection = 0;
            if ( (direction_deg == RIGHT || direction_deg == LEFT)  && useForeAftEncoder)
            {
                lateralCorrection = -(double)foreAftEncoder.getCurrentPosition()/10000;
                RobotLog.d(String.format("lateral Correction:  %.03f",lateralCorrection));
            }

            motorCommands[0] = (Math.sin(direction_rad + (.25 * 3.14)) * speed) - correction + lateralCorrection;//LF_motorPower
            motorCommands[1] = (Math.sin(direction_rad - (.25 * 3.14)) * speed) + correction + lateralCorrection;//RF_motorPower
            motorCommands[2] = (Math.sin(direction_rad - (.25 * 3.14)) * speed) - correction + lateralCorrection;//LR_motorPower
            motorCommands[3] = (Math.sin(direction_rad + (.25 * 3.14)) * speed) + correction + lateralCorrection;//RR_motorPower

            //Need to find max value so we can scale commands because some may be larger than 1.
            //RobotLog.d(String.format("CASH: Motor Commands:  %.03f",motorCommands[0]));
            double maxCommandValue = abs(motorCommands[0]);

            for(int i=1;i < motorCommands.length;i++)
            {
                if(abs(motorCommands[i]) > maxCommandValue)
                {
                    maxCommandValue = abs(motorCommands[i]);
                }
//            RobotLog.d(String.format("CASH: Max Command Value:  %.03f",maxCommandValue));
            }

            LFMP = motorCommands[0];
            RFMP = motorCommands[1];
            LRMP = motorCommands[2];
            RRMP = motorCommands[3];
            if (maxCommandValue > 1 )
            {
                LFMP = motorCommands[0]/maxCommandValue;
                RFMP = motorCommands[1]/maxCommandValue;
                LRMP = motorCommands[2]/maxCommandValue;
                RRMP = motorCommands[3]/maxCommandValue;
            }
            leftFrontMotor.setPower(LFMP);
            leftRearMotor.setPower(LRMP);
            rightFrontMotor.setPower(RFMP);
            rightRearMotor.setPower(RRMP);

        }
        this.Stop();
        return foundFreight;

    }

    //This is a method to rotate the robot.  The input is the angle in degrees to rotate the robot
    // relative to the playing field.  If the robot needs to turn right, the desired direction is
    // degrees (z is up and the right hand rule defines the sign).
    public void RotateRobotAuto(double direction,double desiredRotation_deg, double pwr)
    {
        double LFMP, RFMP, LRMP, RRMP, correction;
        imu.resetAngle();
        imu.desiredGlobalAngle_d = direction*desiredRotation_deg;

        double cmdPower = direction*pwr;
        LFMP = - cmdPower;//LF_motorPower
        RFMP = + cmdPower;//RF_motorPower
        LRMP = - cmdPower; //LR_motorPower
        RRMP = + cmdPower; //RR_motorPower

        leftFrontMotor.setPower(LFMP);
        leftRearMotor.setPower(LRMP);
        rightFrontMotor.setPower(RFMP);
        rightRearMotor.setPower(RRMP);
        boolean angleAchieved = false;
        while (this.IsOpModeActive() && !angleAchieved)
        {

            if (direction > 0) {
                //RobotLog.i(String.format("Turning Right.  AngleError:  %.03f",  imu.getAngle()));
                if (imu.getAngle() <= 0) {
                    angleAchieved = true;
                    //this.Stop();
                }
            }
            if (direction < 0) {
                //RobotLog.i(String.format("Turning Left. AngleError:  %.03f",imu.getAngle()));
                if ( imu.getAngle() >= 0) {
                    angleAchieved = true;
                    //this.Stop();
                }
            }
        }
        this.Stop();
    }

    public void RotateRobotAuto2(double direction,double desiredRotation_deg, double pwr)
    {
        double LFMP, RFMP, LRMP, RRMP, correction;
//        imu.resetAngle();
        imu.desiredGlobalAngle_d = direction*desiredRotation_deg;

        //Need to limit power for rotation due to overshoot.  A command of 1 will overshoot 90 degees by 15-20 degrees
        if (pwr >.5){
            pwr = .5;
        }

        double cmdPower = direction*pwr;

        boolean angleAchieved = false;
        double currentAngle;
        while (this.IsOpModeActive() && !angleAchieved)
        {
            if (direction > 0) {
                if ( imu.getAngle() < 40){
                    cmdPower = direction*.1;
                }
                if ( imu.getAngle() <= 0) {
                    angleAchieved = true;
                    this.Stop();
                }
//                RobotLog.i(String.format("Turning Right.  AngleError:  %.03f Power Is: %.03f",  imu.getAngle(),cmdPower));
            }
            if (direction < 0) {

                if ( imu.getAngle() > -40){
                    cmdPower = direction*.1;
                }
                if (  imu.getAngle() >= 0) {
                    angleAchieved = true;
                    this.Stop();
                }
//                RobotLog.i(String.format("Turning Left. AngleError:  %.03f Power Is: %.03f",imu.getAngle(),cmdPower));
            }
            LFMP = - cmdPower;//LF_motorPower
            RFMP = + cmdPower;//RF_motorPower
            LRMP = - cmdPower; //LR_motorPower
            RRMP = + cmdPower; //RR_motorPower

            leftFrontMotor.setPower(LFMP);
            leftRearMotor.setPower(LRMP);
            rightFrontMotor.setPower(RFMP);
            rightRearMotor.setPower(RRMP);
        }
        this.Stop();
    }

    public void navigateAprilTags(double pwr){
        double LFMP, RFMP, LRMP, RRMP, correction;
        double cmdPower = pwr;
        while (this.IsOpModeActive() )
        {
            LFMP = - cmdPower;//LF_motorPower
            RFMP = + cmdPower;//RF_motorPower
            LRMP = - cmdPower; //LR_motorPower
            RRMP = + cmdPower; //RR_motorPower

            leftFrontMotor.setPower(LFMP);
            leftRearMotor.setPower(LRMP);
            rightFrontMotor.setPower(RFMP);
            rightRearMotor.setPower(RRMP);
        }
        this.Stop();
    }

    private double rateLimitCmds(double raw_value, double prev_cmd,boolean use_rate_limiting, boolean use_exponential_map)
    {


        double sign_of_cmd, sign_of_pre_cmd;

        //Start of the Exponetial Control of robot...smoother at low speeds for all commands
        if (raw_value > 0) {sign_of_cmd = 1;}
        else if (raw_value < 0) {sign_of_cmd = -1;}
        else {sign_of_cmd = 0;}

        if (prev_cmd > 0) {sign_of_pre_cmd = 1;}
        else if (prev_cmd < 0) {sign_of_pre_cmd = -1;}
        else {sign_of_pre_cmd = 0;}

        if (sign_of_cmd != sign_of_pre_cmd && sign_of_pre_cmd != 0) {sign_of_cmd=sign_of_pre_cmd;}
        double smoothedCMD = raw_value;
        if (use_exponential_map) {
            smoothedCMD = (1.2 * Math.pow(1.043, (abs(raw_value) * 100)) - 1.2 + .2 * (abs(raw_value) * 100))/100;
            double smoothedCMD2 = Math.pow(1.05,(Math.abs(raw_value)-1)*100);
            raw_value = sign_of_cmd*smoothedCMD;
            RobotLog.d(String.format("RawCMD %f.03  ExpCMD: %f.03", raw_value,smoothedCMD));

        }
//        RobotLog.d(String.format("RawCMD %f.03  ExpCMD: %f.03", raw_value,smoothedCMD));
        double deltaCMD = raw_value - prev_cmd;
        double rateLimitedCmd = raw_value;

        if (use_rate_limiting) {

            if (prev_cmd > 0 && raw_value > prev_cmd)  //positive accel
            {
                if (abs(raw_value - prev_cmd) > RATELIMIT) {
                    deltaCMD = RATELIMIT;
                }

            } else if (prev_cmd > 0 && raw_value < prev_cmd)  //positive decel
            {
                if (abs(raw_value - prev_cmd) > RATELIMIT) {

                    deltaCMD = -RATELIMIT;
                }
            }

            else if (prev_cmd < 0 && raw_value < prev_cmd)// negative accel
            {
                if (abs(raw_value - prev_cmd) > RATELIMIT) {
                    deltaCMD = -RATELIMIT;
                }

            } else if (prev_cmd < 0 && raw_value > prev_cmd) // negative decel
            {
                if (abs(raw_value - prev_cmd) > RATELIMIT) {
                    deltaCMD = RATELIMIT;
                }

            } else if (prev_cmd == 0 && abs(raw_value) > DEADBAND) {
                if (raw_value > 0) {
                    deltaCMD = RATELIMIT;
                } else {
                    deltaCMD = -RATELIMIT;
                }
            }
            rateLimitedCmd = prev_cmd + deltaCMD;
        }

        return rateLimitedCmd;
    }

}
