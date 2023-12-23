package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name="AutoDistanceBlue2")
public class AutoDistanceBlue2 extends LinearOpMode {

    private DistanceSensor distanceSensor;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor BRight;
    private DcMotor BLeft;


    @Override
    public void runOpMode() {
        float Prop;
        Prop = 0;

        BLeft = hardwareMap.get(DcMotor.class, "BLeft");
        BRight = hardwareMap.get(DcMotor.class, "BRight");
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        FRight = hardwareMap.get(DcMotor.class, "FRight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FRight.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();


        if (opModeIsActive()) {
            BLeft.setPower(0.25);
            BRight.setPower(0.25);
            FRight.setPower(0.25);
            FLeft.setPower(0.25);

            telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "RunningMotorsForwards");
            telemetry.update();

            sleep(1700);

            BLeft.setPower(0);
            BRight.setPower(0);
            FRight.setPower(0);
            FLeft.setPower(0);
            //TODO: Change around these values
            //TODO: FIGURE OUT BETTER WAY TO SOLVE THIS ISSUE WITH INACCURATE SENSOR

            if (distanceSensor.getDistance(DistanceUnit.CM) < 25) {
                sleep(1000);
                Prop = 1;

                telemetry.addData("Prop1", "Prop = 1");
                telemetry.update();

            } else {
                turnLeft(0.25, 1000);
                if (distanceSensor.getDistance(DistanceUnit.CM) < 25) {
                    sleep(1000);
                    Prop = 2;

                    telemetry.addData("Prop2", "Prop = 2");
                    telemetry.update();


                } else {
                    turnRight(0.25, 2500);
                    sleep(1000);
                    Prop = 3;

                }

            }

        }

        if (Prop == 1) {
            PropEqualsOne();
        } else if (Prop == 2) {
            PropEqualsTwo();
        } else if (Prop == 3) {
            PropEqualsThree();
        }
    }

    public void PropEqualsOne () {
        runMotorsTime(0.25, 650);
        runMotorsTime(-0.25, 2400);
        turnRight(0.25, 1950);
        runMotorsTime(0.25,8000);

        telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Status", "Running");
        telemetry.update();

    }
    public void PropEqualsTwo () {
        runMotorsTime(0.25, 900);
        runMotorsTime(-0.25, 900);
        turnRight(0.25,1000);
        runMotorsTime(-0.25,2600);
        turnRight(0.25, 2050);
        runMotorsTime(0.25,8500);
    }
    public void PropEqualsThree () {
        runMotorsTime(0.25, 700);
        runMotorsTime(-0.25, 700);
        turnLeft(0.25,1500);
        runMotorsTime(-0.25,2600);
        turnRight(0.25, 2050);
        runMotorsTime(0.25,8500);
    }



    public void brakeMotors () {
        BLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMotorsTime (double power, long motorTime){
        BLeft.setPower(power);
        BRight.setPower(power);
        FRight.setPower(power);
        FLeft.setPower(power);

        telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Status", "Running");
        telemetry.update();

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }

    public void strafeMotorsRight (double power, long motorTime){
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

    public void strafeMotorsLeft (double power, long motorTime){
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

    public void turnRight (double power, long motorTime){
        BLeft.setPower(power);
        BRight.setPower(-power);
        FRight.setPower(-power);
        FLeft.setPower(power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);

    }


    public void turnLeft (double power, long motorTime){
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
