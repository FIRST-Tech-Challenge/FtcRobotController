package org.firstinspires.ftc.teamcode.TANVIII;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoNomNom extends LinearOpMode {

    Robot robot;

    //declare all motors
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    /*
    DcMotor armMotor;
    Servo leftyServo;
    Servo rightyServo;
    */

    //wheel measurements
    final double PPR = 537.7;
    final double motorToWheelRatio = 1.4;
    final double wheelDiaMm = 96;
    final double PI = 3.14159;
    final double wheelCircIn = wheelDiaMm* PI / 25.4; //~11.87

    double tickToIn = motorToWheelRatio * wheelCircIn / PPR; // #ticks / tickToInches = #inches
    double inToTick = 33; //PPR / (motorToWheelRatio * wheelCircIn); //~32.357

    public double bigAbsVal(double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }
        return max;
    }

    //TODO: i forgot what this does but should put it in robot class
    private void autoStraight(double straightSpeed, double centerTurnSpeed, double mecanumSpeed) {
        //inches * tickToIn
        /*
        straightSpeed positive is forward
        centerTurnSpeed positive is clockwise turn
        mecanumSpeed positive is right (from back of bot)
         */
        //set power
        double flpr = -(straightSpeed) - (centerTurnSpeed) - (mecanumSpeed);
        double frpr = (straightSpeed) - (centerTurnSpeed) - (mecanumSpeed);
        double blpr = -(straightSpeed) - (centerTurnSpeed) + (mecanumSpeed);
        double brpr = (straightSpeed) - (centerTurnSpeed) + (mecanumSpeed);

        //scaling
        double bigPr = bigAbsVal(flpr, frpr, blpr, brpr);

        if (Math.abs(bigPr) > 1) {
            double scaleFactor = Math.abs(bigPr);
            flpr /= scaleFactor;
            frpr /= scaleFactor;
            blpr /= scaleFactor;
            brpr /= scaleFactor;
        }
        robot.setDrivetrainPower(flpr, frpr, blpr, brpr);
    }

    public void encoderStraight(double inches) {

        double flTicksNow = Math.abs(fl.getCurrentPosition());
        double flTicksEnd = inches * inToTick + flTicksNow; //rip ticksToIn :(

        while (flTicksNow < flTicksEnd && opModeIsActive()) {
            double KP = 0.007; //0.0027;
            flTicksNow = Math.abs(fl.getCurrentPosition());

            double flError = flTicksEnd - flTicksNow;
            if ((flError) < 16) {
                break;
            }

            double flPower = KP * flError;
            if (flPower > 1) {
                flPower = 1;
            } else if (flPower < -1) {
                flPower = -1;
            }
            autoStraight(flPower/4, 0,0);
            flTicksNow = Math.abs(fl.getCurrentPosition());
        }
        autoStraight(0,0,0); //stop
    }
    //calculating KP:
    //KP*(12*intotick) ~= 1
    //KP ~= 1/(# ticks in 12")
    //make KP more than that for cutoff, then cutoff before sending to autostraight
    //KP ~= 1/(12*32.357) ~= 0.0026

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            encoderStraight(24);
            sleep(5);
            break;
        }
    }
}
