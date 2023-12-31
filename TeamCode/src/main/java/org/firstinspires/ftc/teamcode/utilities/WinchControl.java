package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

//Elevator Control:
//  This class defines the controls for the elevator
//  The elevator is the item that will raise and lower the pixles to a level where the "bucket" will rotate
//  to drop the pixles into place  This class defines a servo for operating the bucket rotation as well.

public class WinchControl {

    public OpMode _opMode;
    //This is variable of the motor to control the motor
    public DcMotor winch_motor;
    //This is the Servo object that controls the bucket
    public Servo hookServo;
    //This is the Servo object that contorls the drone launcher
    private double MAX_POWER = 1; //will not let the motor faster than this
    public int RAISE_LIMIT =200000;  // encoder value that is the highest we want the elevator to go.
    private double LOWER_LIMIT = -200000;  // encoder value for the lowest we want the elevator to go.

    //Schultz Additions
    //  Added PID controller that will be used to control the teliop to a position the elevator to desired position
    private pid_controller pos_pid = new pid_controller();  // PID control for position control.
    private double hold_position_setpoint;  //This is the set point to control elevator to get to position.


    //Initialize Elevator that resets the encoder position
    public void init(OpMode opMode, String configName){
        _opMode = opMode;
        winch_motor = _opMode.hardwareMap.get(DcMotor.class, configName);
        winch_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        winch_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hookServo = _opMode.hardwareMap.get(Servo.class, "hook_servo");
    }

    public void init2(OpMode opMode, String configName){
        _opMode = opMode;
        winch_motor = _opMode.hardwareMap.get(DcMotor.class, configName);
        winch_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        winch_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hookServo = _opMode.hardwareMap.get(Servo.class, "hook_servo");

        pos_pid.init_pid(.005,0,0);  //Position controller to control elevator to a specific position.
    }

    public int getCurrentPostion() {
        int winch_motorCurrentPosition = winch_motor.getCurrentPosition();
        return winch_motorCurrentPosition;
    }

    //Elevator Controls for Teliop mode from joystick command
    public void raiseLowerWinch_T(double cmd) {
        int ElevatorPosition = winch_motor.getCurrentPosition();
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

        RobotLog.i(String.format("Winch encoder %d",winch_motor.getCurrentPosition()));
        winch_motor.setPower(elevatorCommand);
    }

    //This function sets what is called the set-point of the elevator.  The set-point is the desired
    //position you would like the elevator to reach.  This is intended be set by the teli op mode for
    //buttons that are preset to command elevaotor to a specific position.
    //NOTE:  YOU MUST USE THE UPDATE METHOD TO ACTIVATE THE PID CONTROLLER TO GO TO THE DESIRED POSITION.
    public void set_winch_desired_position(int hold_set_point){
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

        int elevator_current_position = winch_motor.getCurrentPosition();
        double minCmd=-.75;
        double maxCmd=1;

        double elevCmd = pos_pid.update(
                hold_position_setpoint,
                winch_motor.getCurrentPosition(),
                minCmd,
                maxCmd,
                dt);

        RobotLog.i(String.format("setPoint %.2f, actual postition: %d, dt: %.2f",
                hold_position_setpoint,winch_motor.getCurrentPosition(),dt));

        winch_motor.setPower(elevCmd);
    }

    //sissor lift Controls.
      public void raise_hook() {
        hookServo.setPosition(0);
//       RobotLog.d(String.format("inrobot.dumphookServo"));
    }
    //Reset_hookServo - returns the hookServo to position to receive another pixle
    public void reset_hookServo() {
        hookServo.setPosition(1);
    }

}
