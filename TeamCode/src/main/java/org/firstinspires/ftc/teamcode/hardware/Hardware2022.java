package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This is the Robot class for 2022 FTC Season
 *
 */
public class Hardware2022 {

    static public double ANGULAR_RATE = 2000.0;
    private final double MIN_VELOCITY = 0.1;

    //Adjustable parameters  here.
    private final double CLAW_CLOSED = 1 ;
    private final double CLAW_OPEN = 0.3 ;
    private final double xAxisCoeff = 35.6 ;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 22.8 ;  // How many degrees encoder to turn to run an inch in X Axis

    //Encoder value of VSlide height in Cone mode,
    private final int CONE_SLIDE_LOW = 0 ;
    private final int CONE_SLIDE_MID = 1200 ;
    private final int CONE_SLIDE_HIGH = 2500 ;

    //Encoder value of VSlide height in No Cone mode
    private final int NOCONE_SLIDE_LOW = 0 ;
    private final int NOCONE_SLIDE_MID = 120 ;
    private final int NOCONE_SLIDE_HIGH = 360;



    private boolean debug = true;
    private Telemetry telemetry;

    /**
     * Robot has 2 state,  with a cone , or without a cone
     */
    enum RobotState {
        HasCone,
        NoCone
    }
    public enum SlideHeight {
        Low,
        Mid,
        High
    }

    private SlideHeight currentVSHeight = SlideHeight.Low;

    //Start with no cone.
    RobotState currentState = RobotState.NoCone;

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

    //IMU
    IMU imu =null ;

    public DcMotor vSlide = null;

    public Servo grabberclaw = null;
    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

    private int vsldieInitPosition = 0;
    /**
     * Initialize hardware.
     */
    public void createHardware() {

        wheelFrontRight = hwMap.get(DcMotorEx.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");
        vSlide = hwMap.get(DcMotor.class, "Vertical");


        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        vSlide.setPower(0);

        sensorColor = hwMap.get(ColorSensor.class, "clawdistance");
        sensorDistance = hwMap.get(DistanceSensor.class, "clawdistance");

        grabberclaw = hwMap.get(Servo.class, "grabberclaw");

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
            double velocityCoff = 0 ;
            if (Math.abs(currentPosition - 0 ) <= Math.abs(distance - currentPosition)) {
                velocityCoff = Math.abs( (currentPosition - 0 )/(distance - 0) * 2 );
            } else {
                velocityCoff = Math.abs( (distance - currentPosition)/(distance - 0) *2);
            }
            if ( velocityCoff <= MIN_VELOCITY ) {
                velocityCoff = MIN_VELOCITY;
            }

            telemetry.addLine().addData("[X Position , in the while >]  ", getXAxisPosition());
            telemetry.addLine().addData("[X target Position , in the while >]  ", wheelFrontLeft.getTargetPosition());
            telemetry.addLine().addData("[X veloCoff , in the while >]  ", velocityCoff);
            telemetry.update();

            wheelFrontLeft.setVelocity(-power * Hardware2022.ANGULAR_RATE *velocityCoff );
            wheelBackLeft.setVelocity(power * Hardware2022.ANGULAR_RATE*velocityCoff);
            wheelFrontRight.setVelocity(power * Hardware2022.ANGULAR_RATE*velocityCoff);
            wheelBackRight.setVelocity(-power * Hardware2022.ANGULAR_RATE*velocityCoff);

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

        telemetry.addLine().addData("[Start Heading: >]  ", startHeading);
        telemetry.addLine().addData("[End Heading: >]  ", endHeading);
        telemetry.update();

        double difference = regulateDegree( endHeading  - currentHeading );

        while ( Math.abs( difference )> 2  ) {

            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            difference = regulateDegree( endHeading  - currentHeading );

            telemetry.addLine().addData("[Heading: Inside While >]  ", currentHeading);
            telemetry.addLine().addData("[Difference:  >]  ", difference);
            telemetry.update();

            if ( difference > 0 ) {
                wheelFrontLeft.setVelocity(-0.3 * Hardware2022.ANGULAR_RATE);
                wheelBackLeft.setVelocity(-0.3 * Hardware2022.ANGULAR_RATE);
                wheelFrontRight.setVelocity(0.3 * Hardware2022.ANGULAR_RATE);
                wheelBackRight.setVelocity(0.3 * Hardware2022.ANGULAR_RATE);
            } else if ( difference < 0) {
                wheelFrontLeft.setVelocity(0.3 * Hardware2022.ANGULAR_RATE);
                wheelBackLeft.setVelocity(0.3 * Hardware2022.ANGULAR_RATE);
                wheelFrontRight.setVelocity(-0.3 * Hardware2022.ANGULAR_RATE);
                wheelBackRight.setVelocity(-0.3 * Hardware2022.ANGULAR_RATE);
            }

        }

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

    /**
     * This method checks current state of robot.
     *
     * @return  Enumeration of robot state.
     */
    RobotState checkState(){
        return currentState;
    }


    /**
     * This method to check if there is a cone close to the claw,
     * If so, close the claw, and change current stats to has Cone.
     *
     */
    public void checkAndGrabCone ( ) {

        //Only try to grab cone if in No Cone state.
        if ( currentState.equals(RobotState.NoCone)){
            if ( debug) {
                telemetry.addLine().addData("[>]  ", "No cone, checking cone.");
                telemetry.update();
            }

            // Check if there is a cone close by, if so close claw
            if (sensorDistance.getDistance(DistanceUnit.CM) < 2) {
                if (debug) {
                    telemetry.addLine().addData("[>]  ", "Found cone,close claw.");
                    telemetry.update();
                }
                grabberclaw.setPosition(CLAW_CLOSED);
                currentState = RobotState.HasCone;
            }

        }
    }

    private int getVSlidePosition () {
        return vSlide.getCurrentPosition() - vsldieInitPosition;
    }

    /**
     *  This operation to raise Vertical Slide to one level higher
     */
    public void raiseVerticalSlide (  ) {
        telemetry.addLine().addData("[ >]  ", "Slide Raising, current high  " + currentVSHeight);
        telemetry.update();

        switch ( currentVSHeight) {
            case Low: {
                if (currentState.equals(RobotState.HasCone)) {
                    while (getVSlidePosition() < CONE_SLIDE_MID) {
                        vSlide.setPower(1);
                    }
                } else if (currentState.equals(RobotState.NoCone)) {
                    while (getVSlidePosition() < NOCONE_SLIDE_MID) {
                        vSlide.setPower(1);
                    }
                }
            }
                break;

            case Mid: {

                if (currentState.equals(RobotState.HasCone)) {
                    while (getVSlidePosition() < CONE_SLIDE_HIGH) {
                        vSlide.setPower(1);
                    }
                } else if (currentState.equals(RobotState.NoCone)) {
                    while (getVSlidePosition() < NOCONE_SLIDE_HIGH); {
                        vSlide.setPower(1);
                    }
                }
                break;
            }

            case High: {
                //DO noting, already highest

            }
        }


    }

    /**
     * This operation to lower vertical Slide to one level lower.
     */
    public void lowerVerticalSlide () {

        switch ( currentVSHeight) {
            case Low: {
                //DO nothing, already lowest.
                break;
            }

            case Mid: {
                if (currentState.equals(RobotState.HasCone)) {
                    while (getVSlidePosition() > CONE_SLIDE_LOW) {
                        vSlide.setPower(-1);
                    }
                } else if (currentState.equals(RobotState.NoCone)) {
                    while (getVSlidePosition() > NOCONE_SLIDE_LOW) {
                        vSlide.setPower(-1);
                    }
                }
                break;
            }

            case High: {
                if (currentState.equals(RobotState.HasCone)) {
                    while (getVSlidePosition() > CONE_SLIDE_MID) {
                        vSlide.setPower(-1);
                    }
                } else if (currentState.equals(RobotState.NoCone)) {
                    while (getVSlidePosition() > NOCONE_SLIDE_MID) {
                        vSlide.setPower(-1);
                    }

                }
            }
        }

    }

    /**
     * Lower vertical Slide freely , using game control
     * @param power, Expect positive input
    public void freeLowerVerticalSlide( float power ) {
        telemetry.addLine().addData("Encoder Reading", vSlide.getCurrentPosition() );
        telemetry.update();

        if (vSlide.getCurrentPosition() > CONE_SLIDE_LOW ) {
            vSlide.setPower( -power );
        }
        else {
            vSlide.setPower ( 0 );
        }

    }
     */

    /**
     * Move vertical Slide freely , using game control
     * @param power
     */
    public void freeMoveVerticalSlide(float power ) {
        //telemetry.addLine().addData("Encoder Reading", vSlide.getCurrentPosition() );
        //telemetry.addLine().addData("pwer input", power );

        telemetry.update();

        if ( ( (vSlide.getCurrentPosition() - vsldieInitPosition)  <= CONE_SLIDE_HIGH
                && power > 0 )
               ||  ( (vSlide.getCurrentPosition() - vsldieInitPosition)  >= 0 )
                && power < 0 )
        {
            //telemetry.addLine().addData("We have power!", power );
            //telemetry.update();
            vSlide.setPower( power );
            //Thread.sleep(100);
        } else {
            vSlide.setPower( 0 );
        }

    }


    public void  releaseCone( ){
        telemetry.addLine().addData("Release", CLAW_OPEN );
        telemetry.update();

        grabberclaw.setPosition(CLAW_OPEN);
        currentState = RobotState.NoCone;

    }



    public void goToHeight ( SlideHeight height ) {
        int targetPosition = 0;

        if (currentState.equals(RobotState.HasCone)) {
            if (height.equals(SlideHeight.Low)) {
                targetPosition = CONE_SLIDE_LOW;

            }
            if (height.equals(SlideHeight.Mid)) {
                targetPosition = CONE_SLIDE_MID;
            }
            if (height.equals(SlideHeight.High)) {
                targetPosition = CONE_SLIDE_HIGH;
            }

        } else if (currentState.equals(RobotState.NoCone)) {
            if (height.equals(SlideHeight.Low)) {
                targetPosition = NOCONE_SLIDE_LOW;
            }
            if (height.equals(SlideHeight.Mid)) {
                targetPosition = NOCONE_SLIDE_MID;
            }
            if (height.equals(SlideHeight.High)) {
                targetPosition = NOCONE_SLIDE_HIGH;
            }

        }

        //Move the slide
        int currentPosition = vSlide.getCurrentPosition();

        if ((currentPosition - targetPosition) > 0 ) {
            //Lower slide
            while (vSlide.getCurrentPosition() > targetPosition ) {
                vSlide.setPower(-1);
            }
        } else {
            //raise slide
            while (vSlide.getCurrentPosition() < targetPosition ) {
                vSlide.setPower(1);
            }

        }
        currentVSHeight = height;


    }

    public void manualgrab() {
        telemetry.addLine().addData("Manaul Grap", CLAW_CLOSED );
        telemetry.update();

        grabberclaw.setPosition(CLAW_CLOSED);
        currentState = RobotState.HasCone;

    }




}
