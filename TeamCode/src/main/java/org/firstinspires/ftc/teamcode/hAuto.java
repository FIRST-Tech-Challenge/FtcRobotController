package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class hAuto extends OpMode {
    double cpr = 537.7;
    double gearRatio = 1;
    double diameter = 3.77;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.7;
    double conversion = cpi * bias;

    private DcMotor middleMotor; // location 0
    private DcMotor leftMotor; // location 1
    private DcMotor rightMotor; // location 2
    private Servo servoClaw;
    private ElapsedTime newTimer = new ElapsedTime();
    private boolean fiveSeconds = false;

    @Override
    public void init() {

        middleMotor = hardwareMap.get(DcMotor.class, "middleMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void start() {

        newTimer.reset();

    }

    @Override
    public void loop() {
        if (!fiveSeconds && newTimer.seconds() >= 5){
            moveToPos(36,0.2);
            telemetry.addData("something","test");
            telemetry.update();
            fiveSeconds = true;
        }

        telemetry.addData("time", newTimer.seconds());
        telemetry.update();
    }
    @Override
    public void stop() {
    }

    public void moveToPos(double inches, double speed) {
        int move = (int)(Math.round(inches * conversion));

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() - move);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() - move);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        while(leftMotor.isBusy() && rightMotor.isBusy()){

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}