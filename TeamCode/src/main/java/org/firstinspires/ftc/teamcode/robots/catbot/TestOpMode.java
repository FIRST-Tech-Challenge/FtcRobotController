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
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotorEx elevator = null;
    private Servo claw = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // mecanum types
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
    //number variables
    private static final float DEADZONE = .1f;
    private static final int MAXELEVTICS = 4320;
    private static final int MINELEVTICS = 0;
    private int currElevTics = 0;
    private final double MOTORSTALLVALUE = .7;
    //bolean variables
    private boolean calibrate = false;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing " + this.getClass() + "...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();
        motorInit();
    }
    @Override
    public void init_loop()
    {
        if(!calibrate)
            calib();
    }
    @Override
    public void loop() {
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
    @Override
    public void stop(){
        calib();
    }
    public void elevatorMove()
    {
        if(gamepad1.dpad_down) {
            calibrate = false;
        }
        if(!calibrate)
            calib();
        else {
            elevator.setPower(1);
            telemetry.addData("elevator position: ", elevator.getCurrentPosition());
            if (gamepad1.right_trigger > DEADZONE) {
                if (currElevTics < MAXELEVTICS - 150)
                    elevator.setTargetPosition(currElevTics + 150);
                else
                    elevator.setTargetPosition(MAXELEVTICS);
            }
            if (gamepad1.left_trigger > DEADZONE) {
                if (currElevTics > MINELEVTICS + 150)
                    elevator.setTargetPosition(currElevTics - 150);
                else
                    elevator.setTargetPosition(MINELEVTICS);
            }
            if (gamepad1.y)
                elevator.setTargetPosition(3983);
            if (gamepad1.b)
                elevator.setTargetPosition(2300);
            if (gamepad1.a)
                elevator.setTargetPosition(100);
            currElevTics = elevator.getCurrentPosition();
        }
    }
    public void clawMove() {
        telemetry.addData("Claw servo position:", claw.getPosition());
        if (gamepad1.left_bumper)
            claw.setPosition(.9);
        if (gamepad1.right_bumper)
            claw.setPosition(0);
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
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setTargetPosition(0);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("elevator position", elevator.getCurrentPosition());
            telemetry.addData("done calibrating", elevator.getCurrentPosition());
        }
    }
    public void motorInit(){
        motorFrontLeft = this.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotor.class, "motorBackRight");
        elevator = this.hardwareMap.get(DcMotorEx.class, "elevator");
        claw = this.hardwareMap.get(Servo.class, "claw");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        this.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.elevator.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        //this.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


