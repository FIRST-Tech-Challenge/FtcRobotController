package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The purpose of this code is to make
 * Boxy run 12 inches FORWARD using the motor encoders
 */

@Autonomous(name="Motor Encoder Class Demo", group="Robot")
public class Motor_Encoder_Class_Demo extends LinearOpMode{
    // Declare Motor Objects
    private DcMotor Motor0 = null;
    private DcMotor Motor1 = null;
    private DcMotor Motor2 = null;
    private DcMotor Motor3 = null;

    // TODO: Declare Motor Encoder Constants
    static final double COUNTS_PER_MOTOR_REV = 537.7; // From spec sheet
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No external gearing
    static final double WHEEL_DIAMETER_INCHES = 3.75; // Measured the wheels
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14);


    @Override
    public void runOpMode(){
        // Hardware Maps
        Motor0 = hardwareMap.get(DcMotor.class, "Motor0");
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");

        // Set Directions
        Motor0.setDirection(DcMotor.Direction.REVERSE);
        Motor1.setDirection(DcMotor.Direction.REVERSE);
        Motor2.setDirection(DcMotor.Direction.FORWARD);
        Motor3.setDirection(DcMotor.Direction.FORWARD);

        // Stop and Reset the Encoders
        Motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the Motors to run using their encoders
        Motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                Motor0.getCurrentPosition(),
                Motor1.getCurrentPosition(),
                Motor2.getCurrentPosition(),
                Motor3.getCurrentPosition());
        telemetry.update();

        waitForStart();

        runTheEncoders(0.5, 48, 48, 1000);
    }

    /**
     * This method will be used to run the motor encoders
     * @param speed
     * @param leftInches
     * @param rightInches
     * @param timeOutS
     */
    public void runTheEncoders(double speed, double leftInches, double rightInches, long timeOutS){
        // Declare variables to hold targets (the counts we want the motors to run to)
        int Motor0Target = 0;
        int Motor1Target = 0;
        int Motor2Target = 0;
        int Motor3Target = 0;

        if(opModeIsActive()){
            // Determine the positions the motors have to run to
            Motor0Target = Motor0.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            Motor1Target = Motor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            Motor2Target = Motor2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            Motor3Target = Motor3.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            // Set the motors to those positions
            Motor0.setTargetPosition(Motor0Target);
            Motor1.setTargetPosition(Motor1Target);
            Motor2.setTargetPosition(Motor2Target);
            Motor3.setTargetPosition(Motor3Target);

            // Turn on RUN_TO_POSITION
            Motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power to motors
            Motor0.setPower(speed);
            Motor1.setPower(speed);
            Motor2.setPower(speed);
            Motor3.setPower(speed);

            /*
            Run until
            a. Motors have reached their positions
            b. Timer has run out
            c. User has pressed stop
             */
            while(opModeIsActive() && Motor0.isBusy() && Motor1.isBusy() && Motor2.isBusy() && Motor3.isBusy()){
                // :)
            }

            // Stop the motors
            Motor0.setPower(0.0);
            Motor1.setPower(0.0);
            Motor2.setPower(0.0);
            Motor3.setPower(0.0);

            // Turn off RUN_TO_POSITIONS
            Motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Sleep (optional)
            sleep(timeOutS);
        }
    }
}
