package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name="Spike Zone Only2")
public class RedSpikeZoneOnly2 extends LinearOpMode {

    private DistanceSensor distanceSensor;
    private DcMotor fleft;
    private DcMotor fright;
    private DcMotor bright;
    private DcMotor bleft;



    @Override
    public void runOpMode () {
        int count = 0;
        String zone = "";

        bleft = hardwareMap.get(DcMotor.class, "bleft");
        bright = hardwareMap.get(DcMotor.class, "bright");
        fleft = hardwareMap.get(DcMotor.class, "fleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        bleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fleft.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        runMotorsTime(0.25, 1500);
        brakeMotors();
        turnRight(0.25, 2100);

        while (opModeIsActive()) {
            // Measures Distance
            if (distanceSensor.getDistance(DistanceUnit.INCH) > 15) {
                turnLeft(0.25, 50);
                count = count + 1;
                telemetry.addData("Turn_Num: ", count);

                // Zone 1
            } else if (count > 5 && count < 19) {
                zone = "Zone 1";
                zone1();
                break;


                // Zone 2
            } else if (count > 20 && count < 33) {
                zone = "Zone 2";
                zone2();
                break;


                // Zone 3
            } else if (count > 34) {
                zone = "Zone 3";
                zone3();
                break;
            }
            telemetry.addData("", zone);
            telemetry.update();
        }
    }

    // Gets Pixel on Spike 1
    public void zone1(){
        runMotorsTime(0.25, 1000);
        brakeMotors();
        runMotorsTime(-0.25,1000);
        turnRight(0.25,900);
        runMotorsTime(0.25,1500);


    }

    // Gets Pixel on Spike 2
    public void zone2() {
        runMotorsTime(0.25, 1200);
        brakeMotors();
        runMotorsTime(-0.25,1300);
        turnRight(0.25,2000);

    }

    // Gets Pixel on Spike 3
    public void zone3() {
        runMotorsTime(0.25, 1500);
        brakeMotors();
        runMotorsTime(-0.25,1200);
        turnRight(0.25,900);
        runMotorsTime(-0.25,1500);
        strafeMotorsRight(0.25,500);
        runMotorsTime(0.25,1200);
        turnRight(0.25,1500);

    }

    public void brakeMotors(){
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMotorsTime(double power, long motorTime) {
        bleft.setPower(power);
        bright.setPower(power * 0.95);
        fright.setPower(power * 0.95);
        fleft.setPower(power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);
    }

    public void strafeMotorsRight(double power, long motorTime) {
        bleft.setPower(-power);
        bright.setPower(power);
        fright.setPower(-power);
        fleft.setPower(power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);
    }

    public void strafeMotorsLeft(double power, long motorTime) {
        bleft.setPower(power);
        bright.setPower(-power);
        fright.setPower(power);
        fleft.setPower(-power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);
    }

    public void turnRight(double power, long motorTime) {
        bleft.setPower(power);
        bright.setPower(-power);
        fright.setPower(-power);
        fleft .setPower(power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);

    }


    public void turnLeft(double power, long motorTime) {
        bleft.setPower(-power);
        bright.setPower(power);
        fright.setPower(power);
        fleft.setPower(-power);

        sleep(motorTime);

        bleft.setPower(0);
        bright.setPower(0);
        fright.setPower(0);
        fleft.setPower(0);
    }
}
