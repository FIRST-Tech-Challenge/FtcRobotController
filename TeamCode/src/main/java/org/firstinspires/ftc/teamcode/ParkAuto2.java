
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "ParkAuto", group = "Autonomous")


public class ParkAuto2 extends LinearOpMode {
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor EncoderWheel;

    int TicksPerRotation = 2000;
    double EncoderWheelDiameter = 1.82;
    int StartingPosition;

    Double Lift_power = 0.5;
    DcMotor LiftLeft;
    DcMotor LiftRight;
    Servo ServoDump;

    /* Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        EncoderWheel = hardwareMap.get(DcMotor.class, "EncoderWheel");

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftLeft = hardwareMap.get(DcMotor.class, "LiftLeft");
        LiftRight = hardwareMap.get(DcMotor.class, "LiftRight");

        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        ServoDump = hardwareMap.get(Servo.class, "ServoDump");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        StartingPosition = EncoderWheel.getCurrentPosition();

        waitForStart();
        //get off wall
        FrontLeft.setPower(0.6);
        FrontRight.setPower(-0.6);
        BackLeft.setPower(0.6);
        BackRight.setPower(-0.6);
        sleep(500);

        //dump auto 1
        MoveToDumpTower();
        ExtendLift();
        DumpBucket();
        RetractLift();

        MoveToDumpTower();

        //park in observation zone
        MoveToCage();
    }

    //NOTE: park in observation zone
    private void MoveToCage(){
        //turn
        FrontLeft.setPower(-0.2);
        FrontRight.setPower(0.2);
        BackLeft.setPower(-0.2);
        BackRight.setPower(0.2);

        sleep(500);

        //go forward
        FrontLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(0.5);

        double initialposition = getDistance();

        while (getDistance() - initialposition < 1.5){
            //stop
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }
    }


    //NOTE:low lift
    private void RetractLift () {

        int currentLiftPosition = LiftRight.getCurrentPosition();

        LiftLeft.setPower(Lift_power);
        LiftRight.setPower(Lift_power);

        while (currentLiftPosition < -100) {
            telemetry.addData("LiftRightPosition:", currentLiftPosition);
            telemetry.update();

            sleep(100);
            currentLiftPosition = LiftRight.getCurrentPosition();
            }

        LiftLeft.setPower(0);
        LiftRight.setPower(0);
    }

    //NOTE:dump
    private void DumpBucket(){

        double startPosition = ServoDump.getPosition();
        double currentPosition = startPosition;

        ServoDump.setPosition(0.25);

        while (currentPosition < 0.24 || currentPosition > 0.26){
            sleep(100);
            currentPosition = ServoDump.getPosition();
            }

        sleep(3000);
        ServoDump.setPosition(startPosition);

        while (currentPosition < startPosition - 0.01 || currentPosition > startPosition + 0.01){
            sleep(100);
            currentPosition = ServoDump.getPosition();
        }
    }

    //NOTE: max lift
    private void ExtendLift(){

        //NOTE; this function moves the lift to max position

        int currentLiftPosition = LiftRight.getCurrentPosition();

        LiftLeft.setPower(-Lift_power);
        LiftRight.setPower(-Lift_power);

        //NOTE; lift position is reversed, a number closer to zero is in the down position.
        //a number with a negative value is extended position.
        while (currentLiftPosition > -2940) {
            telemetry.addData("LiftRightPosition:", currentLiftPosition);
            telemetry.update();

            sleep(100);
            currentLiftPosition = LiftRight.getCurrentPosition();
        }

        LiftLeft.setPower(0);
        LiftRight.setPower(0);
    }

    //NOTE: tower
    private void MoveToDumpTower(){

        double distance = getDistance();

        //go forward
        FrontLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(0.5);

        while (distance <= -3.5) {
            telemetry.addData("Distance: ", distance);
            telemetry.update();

            sleep(100);
            distance = getDistance();
        }

        //stop
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    private double getDistance() {
        double Rotations = (double) (EncoderWheel.getCurrentPosition() - StartingPosition) / TicksPerRotation;
        return Rotations * EncoderWheelDiameter * Math.PI;
    }
}