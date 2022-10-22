package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Lift {
    private Telemetry telemetry;
    public DcMotorEx liftMotor;

    static final double     MM_TO_INCHES = 0.0393700787;

    static final double     COUNTS_PER_MOTOR_REV    = 8192;     // ticks at the motor shaft
    static final double     DRIVE_GEAR_REDUCTION    = 5.23;     // TODO: Fix to 3:1 gear reduction (slowing down)
    static final double     PULLEY_WHEEL_DIAMETER_INCHES   = 24.25 * MM_TO_INCHES ;     // convert mm to inches
    static final double     TICK_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (PULLEY_WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     LIFT_UP_SPEED           = 0.5;
    static final double     LIFT_DOWN_SPEED         = 0.5;

    public final double     MINIMUM_CLEARANCE_HEIGHT    = 43 * MM_TO_INCHES;    // inches to lift to clear side panels

    public final double     LIFT_POSITION_RESET = 0;
    public final double     LIFT_POSITION_GROUND = 10;
    public final double     LIFT_POSITION_LOWPOLE= 20;
    public final double     LIFT_POSITION_MIDPOLE= 30;
    public final double     LIFT_POSITION_HIGHPOLE = 40;
    public final double     LIFT_POSITION_PICKUP = 8;

    public final double     HARD_STOP_CURRENT_DRAW = 100;

    public static double currentLiftHeight;

    public Lift(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        liftMotor = (DcMotorEx) hwMap.dcMotor.get("Lift");

        liftMotor.setPower(LIFT_UP_SPEED);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition () {
        return liftMotor.getCurrentPosition();
    }

    public void raiseHeightTo (double heightInInches) {
        //raising heights to reach different junctions, so four values
        int ticksNeeded = (int)(heightInInches/TICK_PER_INCH) + 1;
        liftMotor.setTargetPosition(ticksNeeded);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isClear () {
        //true means turret can turn and lift is raised to minimum clearance; false is the opposite
        double currentLiftHeight = liftMotor.getCurrentPosition() * TICK_PER_INCH;
        return currentLiftHeight >= MINIMUM_CLEARANCE_HEIGHT;

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
        liftMotor.setPower(power);
    }

}