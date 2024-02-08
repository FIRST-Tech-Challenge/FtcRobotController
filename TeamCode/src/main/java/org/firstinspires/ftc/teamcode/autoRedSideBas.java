package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name="RedSide bas", group="AutoRedSide")
public class autoRedSideBas extends LinearOpMode {

    void goForward(double speed, double tps) {

        motorA.setPower(-speed);
        motorB.setPower(speed);
        waitTime(tps);
        motorA.setPower(0);
        motorB.setPower(0);
    }

    void goLeft() {

        motorA.setPower(0.5);
        motorB.setPower(0.5);
        waitTime(0.59);
        motorA.setPower(0);
        motorB.setPower(0);
    }

    void waitTime(double tps) {
        double t=getRuntime();
        while (getRuntime()-t < tps) {idle();}
    }

    private DcMotor motorA;
    private DcMotor motorB;
    private DcMotorEx bras1;
    private DcMotorEx bras2;

    private Servo coudeG;
    private Servo coudeD;

    private Servo mainG;
    private Servo mainD;
    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "moteur1");
        motorB = hardwareMap.get(DcMotor.class, "moteur2");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coudeG = hardwareMap.get(Servo.class, "coudeG");
        coudeD = hardwareMap.get(Servo.class, "coudeD");
        mainG = hardwareMap.get(Servo.class, "mainG");
        mainD = hardwareMap.get(Servo.class, "mainD");

        double t;



        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
            idle();
        }
        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bras2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Started");
        telemetry.update();

        coudeG.setPosition(0.486);
        coudeD.setPosition(1-0.486);
        mainG.setPosition(0.5);
        mainD.setPosition(0.5);

        waitTime(3);

        goForward(0.5,0.5);

        waitTime(1);

        mainG.setPosition(0);
        mainD.setPosition(1);


        waitTime(1.5);
        coudeG.setPosition(0.91);
        coudeD.setPosition(1-0.91);

        bras1.setPower(-0.3);
        bras2.setPower(-0.3);
        waitTime(0.5);
        bras1.setPower(0);
        bras2.setPower(0);

        goLeft();
        goForward(0.5,5);

        waitTime(3);

        while (opModeIsActive()) {
            coudeG.setPosition(0.91);
            coudeD.setPosition(1-0.91);
            mainG.setPosition(0);
            mainD.setPosition(1);
            telemetry.addData("Status", "Wating for end");
            telemetry.update();
        }
    }
}
