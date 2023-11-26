package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

//This is a basic example of a file that you can create the controls for the function you are designing.
//For example raising and lowering an elevator of some kind.
public class SweeperControl {

    public OpMode _opMode;
    public DcMotor sweeper_motor;   //This is variable of the motor to control the motor
    public DigitalChannel end_of_travel_switch = null;  //An example of a way to stop the function of the motor when it hits a switch
    private double MAX_POWER = 1; //will not let the motor faster than this

    //Initialize Motor
    public void init(OpMode opMode, String configName){
        _opMode = opMode;
        sweeper_motor = _opMode.hardwareMap.get(DcMotor.class, configName);
       sweeper_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    sweeper_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeper_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

       // end_of_travel_switch = _opMode.hardwareMap.get(DigitalChannel.class, "end_of_travel_switch");
       // end_of_travel_switch.setMode(DigitalChannel.Mode.INPUT);
    }

    //Helper for digital sensor
    public boolean endOfTravelLimitSwitch(){
        return !end_of_travel_switch.getState();
    }

    //Elevator Controls
    public void rollSweeperOut_T(double cmd) {
        int SweeperPosition = sweeper_motor.getCurrentPosition();
        double SweeperCommand;

        SweeperCommand = cmd;


    //   RobotLog.i(String.format("Elevator encoder %d",sweeper_motor.getCurrentPosition()));

        sweeper_motor.setPower(SweeperCommand);
    }
    public void operateMotor(double cmd) {
        RobotLog.w(String.format("Sweeper EncoderPosition= %d ",Math.abs(sweeper_motor.getCurrentPosition())));
        sweeper_motor.setPower(Range.clip(cmd,-MAX_POWER,MAX_POWER));
    }

    public void operateMotorAuto(double cmd, int stop_position){
        while (((LinearOpMode) _opMode).opModeIsActive() && (Math.abs(sweeper_motor.getCurrentPosition()) < stop_position)) {
            operateMotor(cmd);
        }
        operateMotor(0);
    }

    public boolean is_motor_name_variable_busy() {
        return sweeper_motor.isBusy();
    }
    public int getCurrentPostion() {return sweeper_motor.getCurrentPosition();}
}
