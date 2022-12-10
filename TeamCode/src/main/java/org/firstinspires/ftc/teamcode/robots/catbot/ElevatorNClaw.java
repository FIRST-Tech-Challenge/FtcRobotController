package org.firstinspires.ftc.teamcode.robots.catbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

class ElevatorNClaw {
    private DcMotorEx elevator = null;
    private Servo claw = null;
    private static final float DEADZONE = .1f;
    private static final int MAXELEVTICS = 4320;
    private static final int MINELEVTICS = 0;
    private static final int ELEVTICKSPOS1 = MINELEVTICS;
    private static final int ELEVTICKSPOS2 = 2300;
    private static final int ELEVTICKSPOS3 = 3983;
    private static final int ELEVTICKSPOS4 = MAXELEVTICS;
    private int currElevTics = 0;
    private double MOTORSTALLVALUE = .7;
    private static boolean calibrate;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private int elevatorSpeed = 150;
    public ElevatorNClaw(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void elevatorMove(double ticks)
    {
        elevator.setPower(1);
        if (ticks > 0) {
            if (currElevTics < MAXELEVTICS - (int)(ticks*elevatorSpeed))
                elevator.setTargetPosition(currElevTics + (int)(ticks*elevatorSpeed));
            else
                elevator.setTargetPosition(MAXELEVTICS);
        }
        if (ticks < 0) {
            if (currElevTics > MINELEVTICS + (int)(ticks*elevatorSpeed))
                elevator.setTargetPosition(currElevTics - (int)(ticks*elevatorSpeed));
            else
                elevator.setTargetPosition(MINELEVTICS);
        }
        currElevTics = elevator.getCurrentPosition();
    }
    public void elevatorMove(char c)
    {
        switch(c)
        {
            case('a'): {elevator.setTargetPosition(ELEVTICKSPOS1);break;}
            case('b'): {elevator.setTargetPosition(ELEVTICKSPOS2);break;}
            case('y'): {elevator.setTargetPosition(ELEVTICKSPOS3);break;}
            case('x'): {elevator.setTargetPosition(ELEVTICKSPOS4);break;}
        }
    }
    public void clawMove(int direction) {
        if (direction > 0)
            claw.setPosition(.5);
        if (direction < 0)
            claw.setPosition(0.2);
    }
    public boolean calib(){
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("elevator calibrating...", elevator.getCurrent(CurrentUnit.AMPS));
        if(elevator.getCurrent(CurrentUnit.AMPS) < MOTORSTALLVALUE)
        {
            elevator.setPower(-.2);
            return false;
        }
        else {
            calibrate = true;
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setTargetPosition(0);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("done calibrating", elevator.getCurrentPosition());
            return true;
        }
    }
    public void telemetryOutput()
    {
        telemetry.addData("Elevator Position \t", elevator.getCurrentPosition());
        telemetry.addData("Claw Position \t", claw.getPosition());
    }
    public void motorInit()
    {
        elevator = this.hardwareMap.get(DcMotorEx.class, "elevator");
        claw = this.hardwareMap.get(Servo.class, "claw");
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.elevator.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public void runToTestEncoders()
    {
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public int getElevatorPosition(){return elevator.getCurrentPosition();}
}
