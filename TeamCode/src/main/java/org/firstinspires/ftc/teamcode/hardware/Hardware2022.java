package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Thread.*;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the Robot class for 2022 FTC Season
 *
 */
public class Hardware2022 {

    //Adjustable parameters  here.
    private final double CLAW_CLOSED = 1.0;
    private final double CLAW_OPEN = 0.7 ;
    private final double xAxisCoeff = 360 ;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 360 ;  // How many degrees encoder to turn to run an inch in X Axis


    private boolean debug = true;
    private Telemetry telemetry;
    /**
     * Robot has 2 state,  with a cone , or without a cone
     */
    enum RobotState {
        HasCone,
        NoCone
    }

    //Start with no cone.
    RobotState currentState = RobotState.NoCone;

    /**
     * Constructor
     * @param m This is the HarewareMap, which is configured on the dirver stataion.
     * @param tm  The Telemetry object, used for debug purpose.
     */
    public Hardware2022(HardwareMap m, Telemetry tm )
    {
        hwMap = m;
        telemetry = tm;
    }

    public HardwareMap hwMap;

    //motors
    public DcMotor wheelFrontRight = null;
    public DcMotor wheelFrontLeft = null;
    public DcMotor wheelBackRight = null;
    public DcMotor wheelBackLeft = null;
    //public DcMotor wheelStrafe = null;

    public DcMotor vertical = null;

    public Servo grabberclaw = null;
    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

    /**
     * Initialize hardware.
     */
    public void createHardware() {

        wheelFrontRight = hwMap.get(DcMotor.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotor.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotor.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotor.class, "lrWheel");
        //wheelStrafe = hwMap.get(DcMotor.class, "wheelStrafe");

        wheelFrontRight.setDirection(DcMotor.Direction.FORWARD);
        wheelBackRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        wheelFrontRight.setPower(0);
        wheelBackRight.setPower(0);
        wheelFrontLeft.setPower(0);
        wheelBackLeft.setPower(0);
        //wheelStrafe.setPower(0);

        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        vertical = hwMap.get(DcMotor.class, "Vertical");
        vertical.setDirection(DcMotor.Direction.FORWARD);
        vertical.setPower(0);

        sensorColor = hwMap.get(ColorSensor.class, "clawdistance");
        sensorDistance = hwMap.get(DistanceSensor.class, "clawdistance");

        grabberclaw = hwMap.get(Servo.class, "grabberclaw");

    }

    /**
     * This operation move robot forward/backward according to the input
     * @param distance  Distance in encoder degree , 360 for a full circle.  Always positive
     * @param power Positive value move forward
     */
    private void moveXAxis( int distance, double power ) {

        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheelFrontRight.setPower(power);
        wheelFrontLeft.setPower(power);
        wheelBackRight.setPower(-power);
        wheelBackLeft.setPower(-power);

        telemetry.addLine().addData("[FL Position >]  ", ""+wheelFrontLeft.getCurrentPosition() );
        telemetry.update();

        while ( Math.abs(wheelFrontLeft.getCurrentPosition()) < distance ) {

            telemetry.addLine().addData("[FL Position >]  ", ""+wheelFrontLeft.getCurrentPosition() );
            telemetry.update();
            try {
                sleep(50);
            } catch (InterruptedException e) {
                telemetry.addLine().addData("[Error  >]  ", e.getMessage() );
                telemetry.update();
            }

        }

        wheelFrontRight.setPower(0);
        wheelFrontLeft.setPower(0);
        wheelBackRight.setPower(0);
        wheelBackLeft.setPower(0);

    }

    /**
     *
     * @param distance  Distance in inches , 360 for a full circle.  Always positive
     * @param power Positive value move forward
     */
    public void moveXAxis( double distance, double power ) {
        moveXAxis( Math.round( (float) distance * this.xAxisCoeff ), power ) ;
    }

    /**
     * This operation move robot lef/right according to the input
     * @param distance  Distance in encoder degree , 360 for a full circle.  Always positive
     * @param power Positive value move right.
     */

    private void moveYAxis( int distance, double power ) {

        wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheelFrontRight.setPower(-power);
        wheelFrontLeft.setPower(power);
        wheelBackRight.setPower(-power);
        wheelBackLeft.setPower(power);

        telemetry.addLine().addData("[FL Position >]  ", ""+wheelFrontLeft.getCurrentPosition() );
        telemetry.update();

        while ( Math.abs(-wheelFrontRight.getCurrentPosition()) < distance ) {

            telemetry.addLine().addData("[FR Position >]  ", ""+wheelFrontRight.getCurrentPosition() );
            telemetry.update();
            try {
                sleep(50);
            } catch (InterruptedException e) {
                telemetry.addLine().addData("[Error  >]  ", e.getMessage() );
                telemetry.update();
            }

        }

        wheelFrontRight.setPower(0);
        wheelFrontLeft.setPower(0);
        wheelBackRight.setPower(0);
        wheelBackLeft.setPower(0);

    }

    /**
     * This operation move robot lef/right according to the input
     * @param distance  Distance inch ,
     * @param power Positive value move right.
     */

    public void moveYAxis( double  distance, double power ) {
        moveYAxis(Math.round((float) distance * yAxisCoeff), power);

    }

    /**
     * This method checks current state of robot.
     *
     * @return  Enumeration of robot state.
     */
    RobotState checkState(){
        //TODO Implment the logic to check here.
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
            //TODO:  Logic here
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

    public void raiseVerticalSlide (  ) {
        telemetry.addLine().addData("[Dummy >]  ", "Slide Raised ");
        telemetry.update();
    }

    public void releaeCone ( ){

        grabberclaw.setPosition(CLAW_OPEN);
        currentState = RobotState.NoCone;


    }

    /**
     * This is the method to move robot state to Has Cone state
     */
    public void moveToHasCone ( ) {
        //TODO: Logic Here
    }

    /**
     *
     */
    public void moveToNoCone () {

    }

}
