package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This is the Robot class for 2022-2023 FTC Season
 *
 */
public class Hardware2022 {

    //This is max wheel and slide motor velocity.
    static public double ANGULAR_RATE = 2300.0;
    private final double MIN_VELOCITY = 0.1;

    //Adjustable parameters  here.
    @Deprecated
    private final double CLAW_CLOSED = 1 ;
    @Deprecated
    private final double CLAW_OPEN = 0.3 ;

    private final double xAxisCoeff = 35.6 ;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 22.8 ;  // How many degrees encoder to turn to run an inch in X Axis

    //Encoder value of VSlide height in Cone mode,
    private final int CONE_SLIDE_LOW = 1600;
    //Get accurate reading for auto
    private final int CONE_SLIDE_MID = 3000;
    private final int CONE_SLIDE_HIGH = 4030;

    private boolean debug = true;
    private Telemetry telemetry;
    private boolean eMode = false;


    //PID control parameter for turning.
    private double kP =0.15;
    private double kI = 0.1;
    private double kD = 0.005;
    private double kF = 0.0;


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

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelFrontLeft.setTargetPosition( distance  );
        wheelBackLeft.setTargetPosition( distance  );
        wheelFrontRight.setTargetPosition( distance  );
        wheelBackRight.setTargetPosition( distance  );

        telemetry.addLine().addData("[Y Position, after setTarget >]  ", getYAxisPosition());
        telemetry.update();

        while ( wheelFrontLeft.isBusy()) {
            telemetry.addLine().addData("[Y Position , in the while >]  ", getYAxisPosition());
            telemetry.addLine().addData("[Y target Position , in the while >]  ", wheelFrontLeft.getTargetPosition());
            telemetry.update();

            wheelFrontRight.setVelocity(power * Hardware2022.ANGULAR_RATE);
            wheelFrontLeft.setVelocity(power * Hardware2022.ANGULAR_RATE);
            wheelBackRight.setVelocity(power * Hardware2022.ANGULAR_RATE);
            wheelBackLeft.setVelocity(power * Hardware2022.ANGULAR_RATE);

        }

        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);


        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
     * @param power Positive value move right, value from 0-1.
     *
     */
    private void moveXAxisDegree(int distance, double power ) {

        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelFrontLeft.setTargetPosition( -distance  );
        wheelBackLeft.setTargetPosition( distance  );
        wheelFrontRight.setTargetPosition( distance  );
        wheelBackRight.setTargetPosition( -distance  );

        telemetry.addLine().addData("[X Position, after setTarget >]  ", getYAxisPosition());
        telemetry.update();

        //Set velocity slow at beginning and end.
        while ( wheelFrontLeft.isBusy()) {
            int currentPosition = getXAxisPosition();

            telemetry.addLine().addData("[X Position , in the while >]  ", getXAxisPosition());
            telemetry.addLine().addData("[X target Position , in the while >]  ", wheelFrontLeft.getTargetPosition());
            telemetry.update();

            wheelFrontLeft.setVelocity(-power * Hardware2022.ANGULAR_RATE  );
            wheelBackLeft.setVelocity(power * Hardware2022.ANGULAR_RATE);
            wheelFrontRight.setVelocity(power * Hardware2022.ANGULAR_RATE);
            wheelBackRight.setVelocity(-power * Hardware2022.ANGULAR_RATE);

        }

        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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



        PIDFController pidfCrtler  = new PIDFController(kP, kI, kD, kF);
        Log.d("9010", "Kp: " + kP + "  kI: " + kI + " kD: " + kD );

        pidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        pidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        pidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        Log.d("9010", "Before entering Loop ");

        while ( !pidfCrtler.atSetPoint()  ) {
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            //Calculate new distance
            difference = regulateDegree(  currentHeading - endHeading );
            double velocityCaculated = pidfCrtler.calculate(difference)/10;

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

        if (power > 0 || (power < 0 && clawTouch.getState() == true)) {
            vSlide.setVelocity(power * ANGULAR_RATE);

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


    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
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
