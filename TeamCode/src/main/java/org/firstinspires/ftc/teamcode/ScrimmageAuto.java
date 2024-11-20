package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous //(name="Robot: Auto Drive By Encoder", group="Robot")
public class ScrimmageAuto extends LinearOpMode {
    ///////////////////////////////pseudocode///////////////////////////////
    //(robot is 17 inches long)
    //move right 24 inches
    //extend linear slide up to second ladder
    //move forward 7 inches(24-17)
    //move slide down slightly (clip specimen on second ladder)
    //release claw
    //move left a few inches (for other team's auto

    ///////////////////////////////code///////////////////////////////
    final private ElapsedTime runtime = new ElapsedTime();
    //constants for inch functions
    static final double ticksPerRev = 1.043;
    static final double wheelDiameter = 3.5;     // For figuring circumference (in inches)
    static final double ticksPerInch  = ticksPerRev / (wheelDiameter * Math.PI);
    //static final double slideTicksPerInch = 1;
    //Motor and servo declaration
    private DcMotor leftBack, rightBack, leftFront, rightFront; //Initializes direct current main wheel motors for the driving function of our robot, gary.
    private DcMotor linearSlide;
    //private Servo hLinearSlide;
    private Servo clawServo;

    @Override
    public void runOpMode() {
        //setting motors and servos
        leftBack    = hardwareMap.get(DcMotor.class, "bl");
        rightBack   = hardwareMap.get(DcMotor.class, "br");
        leftFront   = hardwareMap.get(DcMotor.class, "fl");
        rightFront  = hardwareMap.get(DcMotor.class, "fr");
        linearSlide = hardwareMap.get(DcMotor.class, "ls");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);

        clawServo = hardwareMap.get(Servo.class, "cs");
        waitForStart();

    }
    protected enum dir { // dir is short for direction btw
        LEFT,
        RIGHT,
        FORWARD,
        BACKWARD

    }
    protected void driveInches(float inches, float speed, dir direction, float timeoutS) {
        double lbDir = 1;
        double rbDir = 1;
        double lfDir = 1;
        double rfDir = 1;
        double targetPos;
        double ticks = 0;
        double deltaTime;
        double lastRuntimeSeconds = 0;

        //sets some motors to negative power depending on direction
        switch(direction) {
            case LEFT:
                rbDir = -1.5;
                lfDir = -1.5;
                lbDir = 1.5;
                rfDir = 1.5;
                break;
            case RIGHT:
                lbDir = -1.5;
                rfDir = -1.5;
                lfDir = 1.5;
                rbDir = 1.5;
                break;
            case FORWARD:
                break;
            case BACKWARD:
                lbDir = -1;
                rbDir = -1;
                lfDir = -1;
                rfDir = -1;
                break;
        }
        if(opModeIsActive()) {
            runtime.reset();
            leftBack.setPower(lbDir * speed);
            rightBack.setPower(rbDir * speed);
            leftFront.setPower(lfDir * speed);
            rightFront.setPower(rfDir * speed);
            targetPos = inches * ticksPerInch;

            while(ticks < targetPos) {
                deltaTime = runtime.seconds() - lastRuntimeSeconds;
                lastRuntimeSeconds = runtime.seconds();
                ticks += deltaTime * speed;
                telemetry.addData("currently going", String.valueOf(direction), " to ", ticks);
                telemetry.addData("Current Pos  ", ticks);
                telemetry.addData("target Pos  ", targetPos);
                telemetry.update();

            }
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

        }

        sleep(500);

    }

//moves 2ft per around 1.6s at 0.25 speed
 protected void driveSeconds(double seconds, float speed, dir direction) {
        double lbDir = 1;
        double rbDir = 1;
        double lfDir = 1;
        double rfDir = 1;
        double currentTime = runtime.seconds();
        double endTime = runtime.seconds() + seconds;
        

        //sets some motors to negative power depending on direction
        switch(direction) {
            case LEFT:
                rbDir = -1.5;
                lfDir = -1.5;
                lbDir = 1.5;
                rfDir = 1.5;
                break;
            case RIGHT:
                lbDir = -1.5;
                rfDir = -1.5;
                lfDir = 1.5;
                rbDir = 1.5;
                break;
            case FORWARD:
                break;
            case BACKWARD:
                lbDir = -1;
                rbDir = -1;
                lfDir = -1;
                rfDir = -1;
                break;
        }
        if(opModeIsActive()) {

            leftBack.setPower(lbDir * speed);
            rightBack.setPower(rbDir * speed);
            leftFront.setPower(lfDir * speed);
            rightFront.setPower(rfDir * speed);

            while(currentTime < endTime) {
                telemetry.addData("currently going", String.valueOf(direction), " for ", endTime);
                telemetry.update();
                currentTime = runtime.seconds();
            }
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

        }

        sleep(500);

    }


    protected void moveClaw(boolean clawOpen) {
        if(clawOpen) {
            clawServo.setPosition(1);
        } else {
            clawServo.setPosition(0);
        }
    }

    protected void moveSlide(double pos) {

    }

    //not complete
    /*protected void moveSlide(float inches, float speed, boolean up, float timeoutS) {
        int dir = 1;
        if (!up) {
            dir = -1;
        }
        if (opModeIsActive()) {
            runtime.reset();
            int targetPos = linearSlide.getCurrentPosition() + (int) (inches * slideTicksPerInch);
            linearSlide.setPower(dir * speed);
            while (opModeIsActive() && timeoutS < runtime.seconds() && linearSlide.isBusy()) {
                telemetry.addData("linear slide currently going", up ? "up" : "down", " to ", targetPos);
                telemetry.update();
            }
            linearSlide.setPower(0);
        }
    }*/

     protected void moveSlideSeconds(double seconds, float speed, boolean up) {
        int dir = 1;
        if (!up) {
            dir = -1;
        }
        double currentTime = runtime.seconds();
        double endTime = runtime.seconds() + seconds;
        if (opModeIsActive()) {
            linearSlide.setPower(dir * speed);
            while(currentTime < endTime) {
                telemetry.addData("linear slide currently going", up ? "up" : "down", " to ", endTime);
                telemetry.update();
                currentTime = runtime.seconds();
            }
            linearSlide.setPower(0);
        }
    }

}

