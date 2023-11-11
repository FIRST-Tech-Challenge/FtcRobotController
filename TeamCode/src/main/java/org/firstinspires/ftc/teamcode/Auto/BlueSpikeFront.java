package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name="BlueSpikeFront")
public class BlueSpikeFront extends LinearOpMode {

    private DistanceSensor distanceSensor;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor BRight;
    private DcMotor BLeft;
    private DcMotor Arm;
    private DcMotor Linear;
    private Servo Claw;

    @Override
    public void runOpMode () {
        int count = 0;
        String zone = "";

        BLeft = hardwareMap.get(DcMotor.class, "BLeft");
        BRight = hardwareMap.get(DcMotor.class, "BRight");
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        FRight = hardwareMap.get(DcMotor.class, "FRight");
        Arm = hardwareMap.dcMotor.get("Arm");
        Linear = hardwareMap.dcMotor.get("Linear");
        Claw = hardwareMap.servo.get("Claw");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        BRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        //Starting Object Detection

        runMotorsTime(-0.25, 1500);
        brakeMotors();
        turnRight(0.25, 1750);

        while (opModeIsActive()) {
            //Check Where Object Is
            if (count > 5 && count < 16) {
                zone = "Zone 1";
                zone1();
                startPark();
                break;
            } else if (count > 17 && count < 31) {
                zone = "Zone 2";
                zone2();
                startPark();
                break;
            } else if (count > 32) {
                zone = "Zone 3";
                zone3();
                startPark();
                break;
            }else{
                turnLeft(0.25, 50);

                //Creating Zones
                count = count + 1;
                telemetry.addData("Turn_Num: ", count);
            }
            telemetry.addData("", zone);
            telemetry.update();
        }
    }

    public void zone1(){
        turnLeft(0.25, 250);
        runMotorsTime(-0.25, 1000);
        brakeMotors();
        turnLeft(0.25, 800);
        runMotorsTime(0.25, 1300);
        brakeMotors();
    }

    public void zone2() {
        runMotorsTime(-0.25, 1150);
        brakeMotors();
        runMotorsTime(0.25, 2000);
        brakeMotors();
    }

    public void zone3() {
        turnLeft(0.25, 100);
        runMotorsTime(-0.25, 1000);
        brakeMotors();
        turnRight(0.25,900);
        runMotorsTime(0.25,1300);
        brakeMotors();
    }

    public void startPark(){
        runMotorsTime(-0.25, 75);
        strafeMotorsLeft(0.25, 1300);
        brakeMotors();
    }

    public void brakeMotors(){
        BLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMotorsTime(double power, long motorTime) {
        BLeft.setPower(power);
        BRight.setPower(power);
        FRight.setPower(power);
        FLeft.setPower(power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }

    public void strafeMotorsRight(double power, long motorTime) {
        BLeft.setPower(-power);
        BRight.setPower(power);
        FRight.setPower(-power);
        FLeft.setPower(power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }

    public void strafeMotorsLeft(double power, long motorTime) {
        BLeft.setPower(power);
        BRight.setPower(-power);
        FRight.setPower(power);
        FLeft.setPower(-power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }

    public void turnRight(double power, long motorTime) {
        BLeft.setPower(power);
        BRight.setPower(-power);
        FRight.setPower(-power);
        FLeft .setPower(power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);

    }


    public void turnLeft(double power, long motorTime) {
        BLeft.setPower(-power);
        BRight.setPower(power);
        FRight.setPower(power);
        FLeft.setPower(-power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }
}
