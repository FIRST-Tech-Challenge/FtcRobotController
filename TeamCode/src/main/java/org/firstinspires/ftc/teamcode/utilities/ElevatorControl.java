package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import static java.lang.Math.*;

//Elevator Control:
//  This class defines the controls for the elevator
//  The elevator is the item that will raise and lower the pixles to a level where the "bucket" will rotate
//  to drop the pixles into place  This class defines a servo for operating the bucket rotation as well.

public class ElevatorControl {

    public OpMode _opMode;
    //This is variable of the motor to control the motor
    public DcMotor elevator_motor;
    //This is the Servo object that controls the bucket
    public Servo bucket;
    //This is the Servo object that contorls the drone launcher
    public Servo launch;
    //This is the digital switch that when pressed will reset the encoder position to 0.
    //This is not implemented yet!
    public DigitalChannel end_of_travel_switch = null;
    private double MAX_POWER = 1; //will not let the motor faster than this
    private double RAISE_LIMIT =2130;  // encoder value that is the highest we want the elevator to go.
    private double LOWER_LIMIT = 0;  // encoder value for the lowest we want the elevator to go.

    //This is the area that we create the snubbing of the end of travel for the elevator.
    //Snubbing is slowing the motor close to the end of travel so that we don't slam the elevator to the ends.
    private double UPPERSNUB = 1840; // Encoder value when we slow the motor down when extending to max
    private double UPPERSNUBFACTOR = 0.5; // The factor that we slow the command down by when extending
    private double LOWERSNUB = 500; // Encoder value when we slow the motor down when retracting
    private double LOWERSNUBFACTOR = 0.5;// The factor that we slow the command down by when retracting

    private int low_pos = 500;
    private int mid_pos = 1000;
    private int high_pos = 2000;

    //Schultz Additions
    //  Added PID controller that will be used to control the teliop to a position the elevator to desired position
    private pid_controller pos_pid = new pid_controller();  // PID control for position control.
    private double hold_position_setpoint;  //This is the set point to control elevator to get to position.


    //Initialize Elevator that resets the encoder position
    public void init(OpMode opMode, String configName){
        _opMode = opMode;
        elevator_motor = _opMode.hardwareMap.get(DcMotor.class, configName);
        elevator_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucket = _opMode.hardwareMap.get(Servo.class, "bucket_servo");
        launch = _opMode.hardwareMap.get(Servo.class, "launch_servo");
    }

    public void init2(OpMode opMode, String configName){
        _opMode = opMode;
        elevator_motor = _opMode.hardwareMap.get(DcMotor.class, configName);
        elevator_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bucket = _opMode.hardwareMap.get(Servo.class, "bucket_servo");
        launch = _opMode.hardwareMap.get(Servo.class, "launch_servo");

        pos_pid.init_pid(.005,0,0);  //Position controller to control elevator to a specific position.
    }


    //Elevator Controls for Teliop mode from joystick command
    public void raiseLowerElevator_T(double cmd) {
        int ElevatorPosition = elevator_motor.getCurrentPosition();
        double elevatorCommand;
        //If command is to raise elevator and we are past the limit of the extention, set cmd to zero
        //else if command is to lower and less that zero set command to zero.
        //otherwise just set the command to what is passed in.
        if (ElevatorPosition > RAISE_LIMIT && cmd > 0) {
            elevatorCommand = 0;
        }
        else if (ElevatorPosition < LOWER_LIMIT  && cmd <0 ) {
            elevatorCommand = 0;
        } else {
            elevatorCommand = cmd;
        }

        //Snubbing functionality.
        //If cmd is extend and we are above the Upper snub encoder position, mult the cmd by the snub factor
        if (ElevatorPosition > UPPERSNUB && cmd > 0) {
          elevatorCommand = UPPERSNUBFACTOR * elevatorCommand;
        }

        //If cmd is to retract and below the Lower snub encodeer psition, mutl the cmd by down snub factor
        if (ElevatorPosition < LOWERSNUB && cmd < 0) {
            elevatorCommand = LOWERSNUBFACTOR * elevatorCommand;
        }
        RobotLog.i(String.format("Elevator encoder %d",elevator_motor.getCurrentPosition()));
        elevator_motor.setPower(elevatorCommand);
    }

    //This method is used to set the position of the elevator in auto mode.
    public void raiseLowerElevatorToPosition_AUTO(double cmd, int desiredPos) {
        while (((LinearOpMode) _opMode).opModeIsActive() && (abs(elevator_motor.getCurrentPosition()) < desiredPos)) {
            this.raiseLowerElevator_T(cmd);
            RobotLog.d(String.format("COMMAND: %.03f DESIRED POSITION :  %d ",cmd,desiredPos));
        }
        this.raiseLowerElevator_T(0);
    }


    public int getCurrentPostion() {
        int elevator_motorCurrentPosition = elevator_motor.getCurrentPosition();
        return elevator_motorCurrentPosition;
    }

    //This function sets what is called the set-point of the elevator.  The set-point is the desired
    //position you would like the elevator to reach.  This is intended be set by the teli op mode for
    //buttons that are preset to command elevaotor to a specific position.
    //NOTE:  YOU MUST USE THE UPDATE METHOD TO ACTIVATE THE PID CONTROLLER TO GO TO THE DESIRED POSITION.
    public void set_elevator_desired_position(int hold_set_point){
        if (hold_set_point >= RAISE_LIMIT){
            hold_position_setpoint = RAISE_LIMIT;
        }else if (hold_set_point <= LOWER_LIMIT){
            hold_position_setpoint = LOWER_LIMIT;
        }else {
            hold_position_setpoint = hold_set_point;
        }
    }

    //This is the update method that is needed to activate the PID to control the elevator to a position.
    //This method is called from teliop mode and you must pass it the elapsed time since the last update.
    //The elapsed time is using the "elapsedTime" class.
    public void updatePosControl(double dt)
    {
//        RobotLog.i(String.format("desPosition %.2f",desPosition));
        //pos_pid.update:  pass the desired position (set-point) and the current encoder position
        //with the dt and max and min commands to the update method will do the math to figure out
        //what the elevCmd needs to be to get to and hold to the desired position.
        double elevCmd = pos_pid.update(
                hold_position_setpoint,
                elevator_motor.getCurrentPosition(),
                -1,
                1,
                dt);

        RobotLog.i(String.format("setPoint %.2f, actual postition: %d, dt: %.2f",
                hold_position_setpoint,elevator_motor.getCurrentPosition(),dt));

        elevator_motor.setPower(elevCmd);
    }

    //Helper for digital sensor
    public boolean endOfTravelLimitSwitch(){
        return !end_of_travel_switch.getState();
    }

    //Pixle Bucket Controls.
    //Dump_pixle - rotates the bucket so pixle drops out of bucket.
    public void dump_pixle() {
        bucket.setPosition(.98);
//       RobotLog.d(String.format("inrobot.dumpbucket"));
    }
    //Reset_bucket - returns the bucket to position to receive another pixle
    public void reset_buckt() {
        bucket.setPosition(-.1);
    }


    //Launching Drone Servo control
    // release - launches the drone
    public void release(){
        launch.setPosition(.2);
    }

    // load - sets the servo to reset position.
    public void load(){
        launch.setPosition(1);
    }
}
