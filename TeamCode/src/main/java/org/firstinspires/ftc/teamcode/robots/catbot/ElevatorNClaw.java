package org.firstinspires.ftc.teamcode.robots.catbot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config("IronGiantElevatorVariables")
class ElevatorNClaw {
    private DcMotorEx elevator = null;
    private Servo claw = null;
    public static float DEADZONE = .1f;
    public static int MAXELEVTICS = 5640;
    public static int MINELEVTICS = 0;
    public static int ELEVTICKSPOS1 = MINELEVTICS + 150;
    public static int ELEVTICKSPOS2 = 2400;
    public static int ELEVTICKSPOS3 = 4083;
    public static int ELEVTICKSPOS4 = MAXELEVTICS;
    public static double CLAWOPEN = .5;
    public static double CLAWCLOSE = .2;
    private int currElevTics = 0;
    public static double MOTORSTALLVALUE = .75;
    private static boolean calibrate;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public static int elevatorSpeed = 300;
    public ElevatorNClaw(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void elevatorMove(double ticks)
    {
        elevator.setPower(1);
        if (ticks > 0) {
            if (currElevTics < MAXELEVTICS - (int)(Math.abs(ticks)*elevatorSpeed))
                elevator.setTargetPosition(currElevTics + (int)(Math.abs(ticks)*elevatorSpeed));
            else
                elevator.setTargetPosition(MAXELEVTICS);
        }
        if (ticks < 0) {
            if (currElevTics > MINELEVTICS + (int)(Math.abs(ticks)*elevatorSpeed))
                elevator.setTargetPosition(currElevTics - (int)(Math.abs(ticks)*elevatorSpeed));
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
            claw.setPosition(CLAWOPEN);
        if (direction < 0)
            claw.setPosition(CLAWCLOSE);
    }
    public boolean calib(){
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw.setPosition(CLAWCLOSE);
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
                                this.elevator.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public int getElevatorPosition(){return elevator.getCurrentPosition();}
    public double getClawPosition(){return claw.getPosition();}
}
