package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This is the Robot class for 2022-2023 FTC Season
 *
 */
public class Hardware2022 {

    //This is max wheel and slide motor velocity.
    static public double ANGULAR_RATE = 1500.0;
    private final double MIN_VELOCITY = 0.1;

    //Adjustable parameters  here.
    @Deprecated
    private final double CLAW_CLOSED = 1 ;
    @Deprecated
    private final double CLAW_OPEN = 0.3 ;

    private final double xAxisCoeff = 216.5 ;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 216.5 ;  // How many degrees encoder to turn to run an inch in Y Axis

    //Encoder value of VSlide height in Cone mode,
    private final int CONE_SLIDE_LOW = 1600;
    //Get accurate reading for auto
    private final int CONE_SLIDE_MID = 3000;
    private final int CONE_SLIDE_HIGH = 4030;

    private boolean debug = true;
    private Telemetry telemetry;
    private boolean eMode = false;


    //PID control parameter for turning.
    private double turnKP = 0.15;
    private double turnKI = 0.1;
    private double turnKD = 0.005;
    private double turnKF = 0.0;
    private double lnKP = 0.15;

    public double getLnKF() {
        return lnKF;
    }

    public void setLnKF(double lnKF) {
        this.lnKF = lnKF;
    }

    public double getLnKD() {
        return lnKD;
    }

    public void setLnKD(double lnKD) {
        this.lnKD = lnKD;
    }

    public double getLnKI() {
        return lnKI;
    }

    public void setLnKI(double lnKI) {
        this.lnKI = lnKI;
    }

    public double getLnKP() {
        return lnKP;
    }

    public void setLnKP(double lnKP) {
        this.lnKP = lnKP;
    }

    private double lnKI = 0.1;
    private double lnKD = 0.005;
    private double lnKF = 0.0;



    public enum SlideHeight {
        Low,
        Mid,
        High,
        Ground
    }

    private SlideHeight currentVSHeight = SlideHeight.Low;

    /**
     * Constructor
     * @param m This is the HardwareMap, which is configured on the driver station.
     * @param tm  The Telemetry object, used for debug purpose.
     */
    public Hardware2022(HardwareMap m, Telemetry tm )
    {
        hwMap = m;
        telemetry = tm;
    }

    public HardwareMap hwMap;

    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;
    public DcMotorEx yEncoder = null;
    public DcMotorEx xEncoder = null;

    //Touch sensor
    DigitalChannel clawTouch ;

    //IMU
    public IMU imu =null ;

    public DcMotorEx vSlide = null;

    //public Servo grabberclaw = null;
    //public ColorSensor sensorColor = null;
    //public DistanceSensor sensorDistance = null;
    public RevTouchSensor touchSensor = null;

    private int vsldieInitPosition = 0;
    /**
     * Initialize hardware.
     */
    public void createHardware() {

        wheelFrontRight = hwMap.get(DcMotorEx.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");
        vSlide = hwMap.get(DcMotorEx.class, "Vertical");
        xEncoder = hwMap.get(DcMotorEx.class, "xEncoder");
        yEncoder = hwMap.get(DcMotorEx.class, "yEncoder");


        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelBackLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackRight.setDirection(DcMotor.Direction.REVERSE);

        vSlide.setDirection(DcMotor.Direction.FORWARD);

        vsldieInitPosition = vSlide.getCurrentPosition() ;

        wheelFrontRight.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackLeft.setVelocity(0);
        vSlide.setVelocity(0);

        /*
        sensorColor = hwMap.get(ColorSensor.class, "clawdistance");
        sensorDistance = hwMap.get(DistanceSensor.class, "clawdistance");

        grabberclaw = hwMap.get(Servo.class, "grabberclaw");
        */
        clawTouch = hwMap.get(DigitalChannel.class, "touch");
        clawTouch.setMode(DigitalChannel.Mode.INPUT);


        //Get IMU.
        imu = hwMap.get(IMU.class, "imu");
        //Our robot mount Conrol hub Logo face backward, and USB port is facing Up.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));


    }

    /**
     * This operation move robot forward/backward according to the input
     * @param distance  Distance in encoder degree , 360 for a full circle.  Positive for forward.
     * @param power Positive value move forward.  Value fom 0 - 1.
     */
    private void moveYAxisDegree(int distance, double power ) {

        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int currenXPosition = xEncoder.getCurrentPosition();
        Log.d("9010", "current X Position " + currenXPosition);

        //Get current orientation.  Angle is between -180 to 180
        int currentPosition = yEncoder.getCurrentPosition();
        int targetPosition = currentPosition + distance;

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        Log.d("9010", "Start Heading " + startHeading);

        Log.d("9010", "Start Position: " + currentPosition );
        Log.d("9010", "End Position: " + targetPosition );


        int difference = distance;
        Log.d("9010", "Difference: " + difference );

        PIDFController lnPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnKp: " + lnKP + "  lnKI: " + lnKI + " lnKD: " + lnKD);
        PIDFController lnXPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnXKp: " + lnKP + "  lnXKI: " + lnKI + " lnXKD: " + lnKD);
        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);


        lnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        lnPidfCrtler.setTolerance(15);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        lnXPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        lnXPidfCrtler.setTolerance(15);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnXPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        turnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        Log.d("9010", "Before entering Loop ");
        double rx;
        double xVelocity;

        long initMill = System.currentTimeMillis();

        while ( !lnPidfCrtler.atSetPoint()
                && ( (System.currentTimeMillis() -initMill  )<5000)  ) {
            currentPosition = yEncoder.getCurrentPosition();
            //Calculate new distance
            difference = currentPosition - targetPosition;
            double velocityCaculated = lnPidfCrtler.calculate(difference)*4;
            if (velocityCaculated > ANGULAR_RATE ) {
                velocityCaculated = ANGULAR_RATE;
            }
            if ( velocityCaculated < -ANGULAR_RATE) {
                velocityCaculated = -ANGULAR_RATE;
            }

            Log.d("9010", "=====================");
            Log.d("9010", "Difference: " + difference);
            Log.d("9010", "Current Position: " + currentPosition );
            Log.d("9010", "Calculated Velocity:  " + velocityCaculated );
            double turnError = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)- startHeading;
            rx = turnPidfCrtler.calculate(turnError)*200;
            Log.d("9010", "Turn Error: " + turnError );
            Log.d("9010", "Calculated rx:  " + rx );

            double xError = xEncoder.getCurrentPosition() - currenXPosition;
            Log.d("9010", "X Error " + xError);
            xVelocity = lnXPidfCrtler.calculate(xError)*3;
            Log.d("9010", "X Vel:  " + xVelocity);


            wheelFrontLeft.setVelocity(velocityCaculated + rx - xVelocity);
            wheelBackLeft.setVelocity(velocityCaculated + rx+ xVelocity);
            wheelFrontRight.setVelocity(velocityCaculated - rx + xVelocity);
            wheelBackRight.setVelocity(velocityCaculated - rx - xVelocity);
        }


        //wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);

    }

    /**
     *
     * @param distance  Distance in inches .  Always positive
     * @param power Positive value move forward
     */
    public void moveYAxis(double distance, double power ) {
        moveYAxisDegree( -(int) Math.round( (float) distance * this.yAxisCoeff ),  -power ) ;
    }

    /**
     * This operation move robot lef/right according to the input
     * @param distance  Distance in encoder degree , 360 for a full circle.  Positive for right.
     * @param power Not used, calculated by PID controller. Kept for backward compatiability
     *
     */
    private void moveXAxisDegree(int distance, double power ) {
        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int currenYPosition = yEncoder.getCurrentPosition();
        Log.d("9010", "current Y Position " + currenYPosition);

        //Get current orientation.  Angle is between -180 to 180
        int currentPosition = xEncoder.getCurrentPosition();
        int targetPosition = currentPosition + distance;

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        Log.d("9010", "Start Heading " + startHeading);

        Log.d("9010", "Start Position: " + currentPosition );
        Log.d("9010", "End Position: " + targetPosition );


        int difference = distance;
        Log.d("9010", "Difference: " + difference );



        PIDFController lnPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnKp: " + lnKP + "  lnKI: " + lnKI + " lnKD: " + lnKD);
        PIDFController lnYPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnYKp: " + lnKP + "  lnYKI: " + lnKI + " lnYKD: " + lnKD);
        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);


        lnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        lnPidfCrtler.setTolerance(15);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        lnYPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        lnYPidfCrtler.setTolerance(15);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnYPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        turnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        Log.d("9010", "Before entering Loop ");
        double rx;
        double yVelocity;

        long initMill = System.currentTimeMillis();

        while ( !lnPidfCrtler.atSetPoint()
                && ( (System.currentTimeMillis() -initMill  )<5000) ) {
            currentPosition = xEncoder.getCurrentPosition();
            //Calculate new distance
            difference = currentPosition - targetPosition;
            double velocityCaculated = lnPidfCrtler.calculate(difference)*4;
            if (velocityCaculated > ANGULAR_RATE ) {
                velocityCaculated = ANGULAR_RATE;
            }
            if ( velocityCaculated < -ANGULAR_RATE) {
                velocityCaculated = -ANGULAR_RATE;
            }

            Log.d("9010", "=====================");
            Log.d("9010", "Difference: " + difference);
            Log.d("9010", "Current Position: " + currentPosition );
            Log.d("9010", "Calculated Velocity:  " + velocityCaculated );
            double turnError = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)- startHeading;
            rx = turnPidfCrtler.calculate(turnError)*200;
            Log.d("9010", "Turn Error: " + turnError );
            Log.d("9010", "Calculated rx:  " + rx );

            double yError = yEncoder.getCurrentPosition() - currenYPosition;
            Log.d("9010", "Y Error " + yError);
            yVelocity = lnYPidfCrtler.calculate(yError)*3;
            Log.d("9010", "Y Veol:  " + yVelocity);


            wheelFrontLeft.setVelocity(-velocityCaculated + rx + yVelocity);
            wheelBackLeft.setVelocity(velocityCaculated + rx+ yVelocity);
            wheelFrontRight.setVelocity(velocityCaculated - rx+ yVelocity);
            wheelBackRight.setVelocity(-velocityCaculated - rx+ yVelocity);
        }


        //wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);
    }


    /**
     * This operation move robot left/right according to the input
     * @param distance  Distance inch , positive to the right.
     * @param power Positive value move right.
     */

    public void moveXAxis(double  distance, double power ) {
        moveXAxisDegree((int) Math.round((float) distance * xAxisCoeff), power);
    }

    private int getXAxisPosition( ) {
        return  wheelFrontLeft.getCurrentPosition() ;
    }

    private int getYAxisPosition( ) {
        return  wheelFrontLeft.getCurrentPosition() ;
    }

    /**
     * Turn robot direction.
     *
     * @param degree  Degrees to turn,  Positive is turn counter clock wise.
     */
    public void turn( double degree) {
        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Get current orientation.  Angle is between -180 to 180
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double startHeading = orientation.getYaw(AngleUnit.DEGREES);
        double endHeading = regulateDegree( startHeading + degree );

        double currentHeading = startHeading;

        Log.d("9010", "Start Heading: " + startHeading );
        Log.d("9010", "End Heading: " + endHeading );


        double difference = regulateDegree( currentHeading - endHeading   );
        Log.d("9010", "Difference: " + difference );



        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "Kp: " + turnKP + "  turnKI: " + turnKI + " turnKD: " + turnKD);

        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        turnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        Log.d("9010", "Before entering Loop ");

        while ( !turnPidfCrtler.atSetPoint()  ) {
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            //Calculate new distance
            difference = regulateDegree(  currentHeading - endHeading );
            double velocityCaculated = turnPidfCrtler.calculate(difference)/10;

            Log.d("9010", "=====================");
            Log.d("9010", "Difference: " + difference);
            Log.d("9010", "Current Heading: " + currentHeading );
            Log.d("9010", "Calculated Volocity:  " + velocityCaculated );

            wheelFrontLeft.setVelocity(velocityCaculated * Hardware2022.ANGULAR_RATE);
            wheelBackLeft.setVelocity(velocityCaculated * Hardware2022.ANGULAR_RATE);
            wheelFrontRight.setVelocity(-velocityCaculated * Hardware2022.ANGULAR_RATE);
            wheelBackRight.setVelocity(-velocityCaculated * Hardware2022.ANGULAR_RATE);
        }


        //wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);
    }

    /**
     * Regulate degreee between -180 and 180, by adding or substracting 360.
     *
     * @param degree
     * @return
     */
    private double regulateDegree ( double degree ) {
       if ( degree > 180) {
           degree -= 360;
       } else if ( degree < -180 ) {
           degree += 360;
       }

       return degree;
    }


    private int getVSlidePosition () {
        return vSlide.getCurrentPosition() - vsldieInitPosition;
    }


    /**
     * Move vertical Slide freely , using game control
     * @param power
     */
    public void freeMoveVerticalSlide(float power ) {
        /*
        telemetry.addLine().addData("Encoder Reading", vSlide.getCurrentPosition() );
        telemetry.addLine().addData("pwer input", power );
        telemetry.update();

         */

        if ( power != 0 ) {
            Log.d("9010", "vSlide position " + vSlide.getCurrentPosition());
        }

        if ( ( (vSlide.getCurrentPosition() - vsldieInitPosition)  <= CONE_SLIDE_HIGH && power > 0 )
                ||  ( (vSlide.getCurrentPosition() - vsldieInitPosition)  >= 0  && power < 0 ) ||  eMode ) {
            //telemetry.addLine().addData("We have power!", power );
            //telemetry.update();
            //Only give power when moving up, or moving down,but touch is not pushed.
            if (power > 0 || (power < 0 && clawTouch.getState() == true)) {
                vSlide.setVelocity(power * ANGULAR_RATE);
            }

            else {
                vSlide.setVelocity(0);
            }
        } else {
            vSlide.setVelocity(0);
        }
        //Thread.sleep(100);

    }


    public void goToHeight ( SlideHeight height ) {
        int targetPosition = 0;

        if (height.equals(SlideHeight.Low)) {
            targetPosition = CONE_SLIDE_LOW;
        }
        if (height.equals(SlideHeight.Mid)) {
            targetPosition = CONE_SLIDE_MID;
        }
        if (height.equals(SlideHeight.High)) {
            targetPosition = CONE_SLIDE_HIGH;
        }
        if (height.equals(SlideHeight.Ground)) {
            targetPosition = vsldieInitPosition;
        }

        //Move the slide
        int currentPosition = vSlide.getCurrentPosition();
        Log.d("9010", "vSlide position before Move: " + vSlide.getCurrentPosition());

        vSlide.setTargetPosition(targetPosition);
        Log.d("9010", "Target position : " + targetPosition);
        vSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int sign = 1;

        if ((currentPosition - targetPosition) > 0 ) {
            sign= -1;
        } else {
            //raise slide
            sign = 1;
        }

        while (vSlide.isBusy()) {
            vSlide.setVelocity( sign * ANGULAR_RATE* 0.5 );
            //Log.d("9010", "Inside Moving Loop : " + vSlide.getCurrentPosition() + " Sign: " + sign);
        }
        vSlide.setVelocity(0);
        Log.d("9010", "after Moving Loop : " + vSlide.getCurrentPosition());
        currentVSHeight = height;
        //Set mode back to Run using encoder.
        vSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double getTurnKP() {
        return turnKP;
    }

    public void setTurnKP(double turnKP) {
        this.turnKP = turnKP;
    }

    public double getTurnKI() {
        return turnKI;
    }

    public void setTurnKI(double turnKI) {
        this.turnKI = turnKI;
    }

    public double getTurnKD() {
        return turnKD;
    }

    public void setTurnKD(double turnKD) {
        this.turnKD = turnKD;
    }

    public double getTurnKF() {
        return turnKF;
    }

    public void setTurnKF(double turnKF) {
        this.turnKF = turnKF;
    }
    /*This method will lower slide until touch sensor gets activated
     */
    public void dropCone() {

        double power = -0.5;
        double slideStartPostion = vSlide.getCurrentPosition();
        double travel = slideStartPostion - vSlide.getCurrentPosition();
        Log.d("9010", "slideStartPostion:  " + slideStartPostion);
        Log.d("9010", "Travel:  " + travel);

        while (clawTouch.getState()==true && ( travel < 900 )) {
            vSlide.setVelocity(power * ANGULAR_RATE);
            travel = slideStartPostion - vSlide.getCurrentPosition();
            //Log.d("9010", "postion:  " + vSlide.getCurrentPosition());
            //Log.d("9010", "Travel:  " + travel);
        }

        Log.d("9010", "After SLide Drop Cone ");
        vSlide.setVelocity(0);

        //Thread.sleep(100);

    }

    public void seteMode ( boolean input ) {
        Log.d("9010", "eMode is set to: " + input );
        if (input) {
            this.eMode = input;
        } else {
            this.eMode = input;
            this.vsldieInitPosition = vSlide.getCurrentPosition();
            Log.d("9010", "Now new vslide init position: " + vsldieInitPosition);
        }
    }

    public boolean iseMode() {
        return eMode;
    }
}
