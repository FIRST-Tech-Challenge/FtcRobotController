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
    private static final int MINELEVTICS = 50;
    private int currElevTics = 0;
    private double MOTORSTALLVALUE = .7;
    private static boolean calibrate;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public ElevatorNClaw(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        if (!calibrate)
            calib();
        else {
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setTargetPosition(0);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void elevatorMove(int ticks)
    {
        elevator.setPower(1);
//        if(gamepad1.dpad_down) {
//            calibrate = false;
//            calib();
//        }
//        else if (calibrate) {
        if (ticks > 0) {
            if (currElevTics < MAXELEVTICS - ticks)
                elevator.setTargetPosition(currElevTics + ticks);
            else
                elevator.setTargetPosition(MAXELEVTICS);
        }
        if (ticks < 0) {
            if (currElevTics > MINELEVTICS + ticks)
                elevator.setTargetPosition(currElevTics - ticks);
            else
                elevator.setTargetPosition(MINELEVTICS);
        }
        currElevTics = elevator.getCurrentPosition();
//        }
    }
    public void clawMove(int direction) {
        if (direction > 0)
            claw.setPosition(.5);
        if (direction < 0)
            claw.setPosition(0.2);
    }
    public void calib(){
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("elevator calibrating...", elevator.getCurrent(CurrentUnit.AMPS));
        if(elevator.getCurrent(CurrentUnit.AMPS) < MOTORSTALLVALUE)
        {
            elevator.setPower(-.2);
        }
        else {
            calibrate = true;
            telemetry.addData("done calibrating", elevator.getCurrentPosition());
        }
    }
    public void motorInit()
    {
        elevator = this.hardwareMap.get(DcMotorEx.class, "elevator");
        claw = this.hardwareMap.get(Servo.class, "claw");
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.elevator.setDirection(DcMotorEx.Direction.REVERSE);
    }
}
