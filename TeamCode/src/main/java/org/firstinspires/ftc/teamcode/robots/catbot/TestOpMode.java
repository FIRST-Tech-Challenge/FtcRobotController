package org.firstinspires.ftc.teamcode.robots.catbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.catbot.util.Utils;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.State;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

@TeleOp(name="Iron Giant OpMode", group="Challenge")
public class TestOpMode extends OpMode {
    private boolean auton = true;
//    private StateMachine autonStateMachine = Utils.getStateMachine(new Stage())
//            .addState(() -> driveToD())
//
////            .addState(() -> driveToD(1))
////            .addState(() -> driveToD(2))
//            .build();
    //variable setup
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotorEx elevator = null;
    private Servo claw = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // motor power
    private double powerElevator = 0;
    private DcMotor motorElevator = null;
    private Servo clawServo = null;
    // regular drive
    // mecanum types
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;

    private double powerClaw = 0;
    //number variables
    private static final float DEADZONE = .1f;
    private static final int MAXELEVTICS = 4320;
    private static final int MINELEVTICS = 0;
    private int currElevTics = 0;
    private double MOTORSTALLVALUE = 12.76;

//    public boolean driveToD() {
//    double powerLeft = 0;
//    double powerRight = 0;
//    boolean done = false;
//        try {
//            TimeUnit.SECONDS.sleep(5);
//        } catch (InterruptedException e) {
//
//        }
//        done = true;
//    while(!done) {
//        powerLeft = 1;
//        powerRight = 1;
//    }
//    if(done) {
//        powerLeft = 0;
//        powerRight = 0;
//        return done;
//    }
//        while(auton) {
//            motorFrontLeft.setPower(powerLeft);
//            motorFrontRight.setPower(powerRight);
//            motorBackRight.setPower(powerRight);
//            motorBackLeft.setPower(powerLeft);
//        }
//    return false;
//    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing " + this.getClass() + "...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();
        elevator = this.hardwareMap.get(DcMotorEx.class, "elevator");
        //calib(); <-- fix!!!!!!!!
        motorInit();
    }

    @Override
    public void loop() {
//        autonStateMachine.execute();

        //tankDrive();
        mechanumDrive();
        elevatorMove();
        clawMove();
    }

    public void tankDrive() {
        powerRight = 0;
        powerLeft = 0;

// tanvi is the bestestestestestest

        if (Math.abs(gamepad1.left_stick_y) > DEADZONE) {
            powerLeft = gamepad1.left_stick_y;
        }
        if (Math.abs(gamepad1.right_stick_y) > DEADZONE) {
            powerRight = gamepad1.right_stick_y;
        }
        motorFrontRight.setPower(powerRight);
        motorFrontLeft.setPower(powerLeft);
        motorBackRight.setPower(powerRight);
        motorBackLeft.setPower(powerLeft);
    }

    public void mechanumDrive() {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x*.65;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        motorFrontLeft.setPower(v1);
        motorFrontRight.setPower(v4);
        motorBackLeft.setPower(v3);
        motorBackRight.setPower(v2);
    }
    public void elevatorMove()
    {
        telemetry.addData("elevator position: ", elevator.getCurrentPosition());
        if(gamepad1.right_trigger > DEADZONE)
        {
            telemetry.addData("right trigger moved", currElevTics);
            telemetry.addData("right trigger moved", MAXELEVTICS);
            telemetry.addData("right trigger moved", elevator.isBusy());
            if(currElevTics < MAXELEVTICS-150)
              elevator.setTargetPosition(currElevTics+150);
            else
              elevator.setTargetPosition(MAXELEVTICS);
         }
        if(gamepad1.left_trigger > DEADZONE)
        {
            if(currElevTics > MINELEVTICS+150)
              elevator.setTargetPosition(currElevTics-150);
            else
              elevator.setTargetPosition(MINELEVTICS);
         }
        if(gamepad1.y)
            elevator.setTargetPosition(3883);
        if(gamepad1.b)
            elevator.setTargetPosition(2300);
        if(gamepad1.a)
            elevator.setTargetPosition(0);
        currElevTics = elevator.getCurrentPosition();
    }
    public void clawMove() {
        telemetry.addData("Claw servo position:", claw.getPosition());
        if (gamepad1.left_bumper)
            claw.setPosition(1);
        if (gamepad1.right_bumper)
            claw.setPosition(.05);
    }
    public void calib(){
        telemetry.addData("elevator calibrating...", elevator.getCurrentPosition());
        //get elevator amperage, probably with a method in dcmotorex instead of the dcmotor class
        //might have to switch to dcmotorex for all of the motors
        //while amperage < stallValue (checks for stall)
        //elevator.setPower(-0.1); or something
        if(elevator.getCurrent(CurrentUnit.AMPS) < MOTORSTALLVALUE)
        {
            elevator.setPower(-.1);
        }
        telemetry.addData("done calibrating", elevator.getCurrentPosition());
    }
    public void motorInit(){
        motorFrontLeft = this.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotor.class, "motorBackRight");
        //elevator = this.hardwareMap.get(DcMotorEx.class, "elevator");
        claw = this.hardwareMap.get(Servo.class, "claw");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        this.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.elevator.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        //this.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        elevator.setPower(1);

    }
}
