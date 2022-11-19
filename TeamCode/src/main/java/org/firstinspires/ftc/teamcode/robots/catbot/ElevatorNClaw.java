package org.firstinspires.ftc.teamcode.robots.catbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ElevatorNClaw {
    private DcMotorEx elevator = null;
    private Servo claw = null;
    private static final float DEADZONE = .1f;
    private static final int MAXELEVTICS = 4320;
    private static final int MINELEVTICS = 50;
    private int currElevTics = 0;
    private double MOTORSTALLVALUE = 1 /*12.76*/;
    private static boolean calibrate;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    public ElevatorNClaw(Telemetry telemetry, Gamepad gamepad, Servo claw, DcMotorEx elevator) {
        this.gamepad1 = gamepad;
        this.telemetry = telemetry;
        this.claw = claw;
        this.elevator = elevator;
        if (!calibrate)
            calib();
        else {
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setTargetPosition(50);
            elevator.setPower(1);
        }
    }
    public void elevatorMove()
    {
        telemetry.addData("elevator position: ", elevator.getCurrentPosition());
        calibrate = false;
        if(gamepad1.right_trigger > DEADZONE)
        {
//            telemetry.addData("right trigger moved", currElevTics);
//            telemetry.addData("right trigger moved", MAXELEVTICS);
//            telemetry.addData("right trigger moved", elevator.isBusy());
            if(currElevTics < MAXELEVTICS-250)
                elevator.setTargetPosition(currElevTics+250);
            else
                elevator.setTargetPosition(MAXELEVTICS);
        }
        if(gamepad1.left_trigger > DEADZONE)
        {
            if(currElevTics > MINELEVTICS+250)
                elevator.setTargetPosition(currElevTics-250);
            else
                elevator.setTargetPosition(MINELEVTICS);
        }
        if(gamepad1.y)
            elevator.setTargetPosition(3883);
        if(gamepad1.b)
            elevator.setTargetPosition(2300);
        if(gamepad1.a)
            elevator.setTargetPosition(50);
        currElevTics = elevator.getCurrentPosition();
    }
    public void clawMove() {
        telemetry.addData("Claw servo position:", claw.getPosition());
        if (gamepad1.left_bumper)
            claw.setPosition(0.7);
        if (gamepad1.right_bumper)
            claw.setPosition(-1);
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
}
