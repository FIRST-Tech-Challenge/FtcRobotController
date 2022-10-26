package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Lift {
    private Telemetry telemetry;
    public DcMotor liftMotor;

    static final double     MM_TO_INCHES = 0.0393700787;

    static final double     COUNTS_PER_MOTOR_REV    = 8192;     // ticks at the motor shaft
    static final double     DRIVE_GEAR_REDUCTION    = 5.23;     // TODO: Fix to 3:1 gear reduction (slowing down)
    static final double     PULLEY_WHEEL_DIAMETER_INCHES   = 24.25 * MM_TO_INCHES ;     // convert mm to inches
    static final double     TICK_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     LIFT_UP_SPEED           = 0.5;
    static final double     LIFT_DOWN_SPEED         = 0.5;

    public final double     MINIMUM_CLEARANCE_HEIGHT    = 43 * MM_TO_INCHES;    // inches to lift to clear side panels

    public final double     LIFT_POSITION_RESET = 0;
    public final double     LIFT_POSITION_GROUND = 12;
    public final double     LIFT_POSITION_LOWPOLE= 14;
    public final double     LIFT_POSITION_MIDPOLE= 30;
    public final double     LIFT_POSITION_HIGHPOLE = 40;
    public final double     LIFT_POSITION_PICKUP = 8;
    public final double     LIFT_ADJUSTMENT = -2;

    public final double     HARD_STOP_CURRENT_DRAW = 100;

    public final String LIFT_SYSTEM_NAME = "Lift";
    public final String LIFT_POLE_LOW = "POLE_LOW";
    public final String LIFT_POLE_MEDIUM = "POlE_MEDIUM";
    public final String LIFT_POLE_HIGH = "POLE_HIGH";
    public final String DELIVERY_HEIGHT = "DELIVERY_HEIGHT";
    public final String PLACEMENT_HEIGHT = "PLACEMENT_HEIGHT";
    public final String LIFT_SUBHEIGHT  = "SUB_HEIGHT";

    public final String TRANSITION_STATE = "TRANSITION";
    public final int DELIVERY_ADJUSTMENT = -3;
    public final double HEIGHT_TOLERANCE  = 0.25;

    public static double currentLiftHeight;



    public Lift(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        liftMotor = hwMap.dcMotor.get("Lift");

//        liftMotor.setPower(LIFT_UP_SPEED);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition () {

        return liftMotor.getCurrentPosition();
    }
    public void setState(String level, String subheight){
        switch(level){
            case LIFT_POLE_LOW:{
                raiseHeightTo(LIFT_POSITION_LOWPOLE + deliveryHeight(subheight));
                break;
            }

        }
    }
    public String getCurrentState() {
        String state = TRANSITION_STATE;
        double currentPosition = getHeightInInches();
        if (inHeightTolerance(currentPosition, LIFT_POSITION_LOWPOLE)) {
            state = LIFT_POLE_LOW;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_MIDPOLE)) {
            state = LIFT_POLE_MEDIUM;
        } else if (inHeightTolerance(currentPosition, LIFT_POSITION_HIGHPOLE)) {
            state = LIFT_POLE_HIGH;
        }
        return state;
    }
    private double getHeightInInches(){

        return ((double) getPosition()/TICK_PER_INCH);
    }
    public int deliveryHeight(String subheight){
        int height = 0;
        if(subheight.equalsIgnoreCase(DELIVERY_HEIGHT)){
            height -= LIFT_ADJUSTMENT;
        }
        return height;
    }

    public void raiseHeightTo (double heightInInches) {
        //raising heights to reach different junctions, so four values
        int ticksNeeded = (int)(heightInInches/TICK_PER_INCH) + 1;
        liftMotor.setTargetPosition(ticksNeeded);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public  boolean isClear () {
        //true means turret can turn and lift is raised to minimum clearance; false is the opposite
        double currentLiftHeight = liftMotor.getCurrentPosition() * TICK_PER_INCH;
        if(currentLiftHeight >= MINIMUM_CLEARANCE_HEIGHT){
            return true;
        }
        return false;

    }
    public void moveToMinHeight(){
        if (!isClear()) {
            raiseHeightTo(MINIMUM_CLEARANCE_HEIGHT);
        }
    }

    public void initializePosition( ) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void setMotor(double power){
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(power);
    }
    private boolean inHeightTolerance(double heightPosition, double poleHeight) {
        return (heightPosition > poleHeight - HEIGHT_TOLERANCE) && (heightPosition < poleHeight + HEIGHT_TOLERANCE);
    }

    }