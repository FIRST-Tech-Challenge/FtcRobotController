package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

//Sweeper Control.  This is a simple control of the sweeper that pulls in pixles
public class SweeperControl {

    public OpMode _opMode;  //Used for Auto code.
    public DcMotor sweeper_motor;   //This is variable of the motor to control the motor
    public DigitalChannel end_of_travel_switch = null;  //An example of a way to stop the function of the motor when it hits a switch
    private double MAX_POWER = 1; //will not let the motor faster than this

    //Initialize Motor:
    //Params:
    // opMode-the opMode object to be used to test if opmode is active for autonomous modes
    // configName - the configuration name used to identify the sensor/actuator within the control/expansion hub
    public void init(OpMode opMode, String configName){
        _opMode = opMode;
        sweeper_motor = _opMode.hardwareMap.get(DcMotor.class, configName);
        sweeper_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sweeper_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeper_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Sweeper Control:  Simply turns on the sweeper based on the command that is passed
    //Params:  cmd - the value of the input desired motor speed.
    public void setSweeperCommand(double cmd) {
        sweeper_motor.setPower(cmd);
    }
}
