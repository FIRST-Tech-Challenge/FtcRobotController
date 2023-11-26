package org.firstinspires.ftc.teamcode.utilities;

import android.app.usage.NetworkStats;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

//This is a basic example of a file that you can create the controls for the function you are designing.
//For example raising and lowering an elevator of some kind.
public class ElevatorControl {

    public OpMode _opMode;
    public DcMotor elevator_motor;
    //This is variable of the motor to control the motor
    public Servo bucket;
    public Servo launch;
    public DigitalChannel end_of_travel_switch = null;  //An example of a way to stop the function of the motor when it hits a switch
    private double MAX_POWER = 1; //will not let the motor faster than this
    private double RAISE_LIMIT =2130;
    private double LOWER_LIMIT = 0;

    private double LOWERSNUB = 500;

    private double UPPERSNUB = 1840;

    private double UPPERSNUBFACTOR = 0.5;

    private double LOWERSNUBFACTOR = 0.5;

    private int low_pos = 500;
    private int mid_pos = 1000;
    private int high_pos = 2000;

    //Schultz Additions
    private pid_controller pos_pid = new pid_controller();
    private double hold_position_setpoint;

    //Initialize Motor
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
//        elevator_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevator_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bucket = _opMode.hardwareMap.get(Servo.class, "bucket_servo");
        launch = _opMode.hardwareMap.get(Servo.class, "launch_servo");

        pos_pid.init_pid(.005,0,0);
    }

    //Helper for digital sensor
    public boolean endOfTravelLimitSwitch(){
        return !end_of_travel_switch.getState();
    }
   public void dump_bucket () {
       bucket.setPosition(.98);
       RobotLog.d(String.format("inrobot.dumpbucket"));
   }
   public void raise_bucket () {
        bucket.setPosition(-.1);
   }

   public void release(){
        launch.setPosition(.2);
   }
    public void load(){
        launch.setPosition(1);
    }

    //Elevator Controls
    public void raiseLowerElevator_T(double cmd) {
        int ElevatorPosition = elevator_motor.getCurrentPosition();
        double elevatorCommand;
        if (ElevatorPosition > RAISE_LIMIT && cmd > 0) {
            elevatorCommand = 0;

        } else if (ElevatorPosition < LOWER_LIMIT  && cmd <0 ) {
            elevatorCommand = 0;
        } else {
            elevatorCommand = cmd;
        }

        if (ElevatorPosition > UPPERSNUB && cmd > 0) {
          elevatorCommand = UPPERSNUBFACTOR * elevatorCommand;
        }

        if (ElevatorPosition < LOWERSNUB && cmd < 0) {
            elevatorCommand = LOWERSNUBFACTOR * elevatorCommand;
        }

        RobotLog.i(String.format("Elevator encoder %d",elevator_motor.getCurrentPosition()));

        elevator_motor.setPower(elevatorCommand);
    }
    public void raiseLowerElevatorToPosition(double cmd, int desiredPos) {
        double elevatorCommand;

        while (((LinearOpMode) _opMode).opModeIsActive() && (Math.abs(elevator_motor.getCurrentPosition()) < desiredPos)) {

            this.raiseLowerElevator_T(cmd);
//            double enc = double(desiredPos)
            RobotLog.d(String.format("COMMAND: %.03f DESIRED POSITION :  %d ",cmd,desiredPos));
        }
        this.raiseLowerElevator_T(0);
//        if (ElevatorPosition > desiredPos && cmd > 0) {
//            elevatorCommand = 0;
//
//        } else if (ElevatorPosition < LOWER_LIMIT  && cmd <0 ) {
//            elevatorCommand = 0;
//        } else {
//            elevatorCommand = cmd;
//        }
//
//        if (ElevatorPosition > UPPERSNUB && cmd > 0) {
//            elevatorCommand = UPPERSNUBFACTOR * elevatorCommand;
//        }
//
//        if (ElevatorPosition < LOWERSNUB && cmd < 0) {
//            elevatorCommand = LOWERSNUBFACTOR * elevatorCommand;
//        }
//
//        RobotLog.i(String.format("Elevator encoder %d",elevator_motor.getCurrentPosition()));
//
//        elevator_motor.setPower(elevatorCommand);
    }
    public void operateMotor(double cmd) {
//        RobotLog.w(String.format("Elevator EncoderPosition= %d ",Math.abs(elevator_motor.getCurrentPosition())));
        elevator_motor.setPower(Range.clip(cmd,-MAX_POWER,MAX_POWER));
    }

    public void operateMotorAuto(double cmd, int stop_position){
       while (((LinearOpMode) _opMode).opModeIsActive() && (Math.abs(elevator_motor.getCurrentPosition()) < stop_position)) {
            operateMotor(cmd);
        }
        operateMotor(0);
    }

    public boolean is_motor_name_variable_busy() {
        return elevator_motor.isBusy();
    }
    public int getCurrentPostion() {return elevator_motor.getCurrentPosition();}


    public void updatePosControl(double dt)
    {
//        RobotLog.i(String.format("desPosition %.2f",desPosition));
        double elevCmd = pos_pid.update(hold_position_setpoint,elevator_motor.getCurrentPosition(),-1,1,dt);
        RobotLog.i(String.format("setPoint %.2f, actual postition: %d, dt: %.2f",
                hold_position_setpoint,elevator_motor.getCurrentPosition(),dt));

        elevator_motor.setPower(elevCmd);
    }
    //Schultz Updates
    public void setHoldPosition(int hold_set_point){
        if (hold_set_point >= RAISE_LIMIT){
            hold_position_setpoint = RAISE_LIMIT;
        }else if (hold_set_point <= LOWER_LIMIT){
            hold_position_setpoint = LOWER_LIMIT;
        }else {
            hold_position_setpoint = hold_set_point;
        }
    }
}
