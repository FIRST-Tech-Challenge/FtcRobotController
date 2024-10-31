package org.firstinspires.ftc.teamcode.autonomous.newAuto.teaching.teach.simpleAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Basic Auto")
public class basicAuto extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lift;

    private CRServo combine;

    private double driveSpeed = 0.5;

    //Test this variable for ALL MOVEMENT, NOT JUST FORWARD
    private int countPer10CM = 0;
    private int countPerCm = countPer10CM/10;

    private void initialize(){
        //DcMotors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        lift = hardwareMap.get(DcMotor.class, "lift");

        //Servos
        combine = hardwareMap.get(CRServo.class, "combine");
    }
    private void setDirection(){
        //Test these
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void setModeTarPos(){
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void resetEncoder(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void setSpeed(double FLS, double FRS, double BRS, double BLS){
        frontLeft.setPower(FLS);
        frontRight.setPower(FRS);
        backLeft.setPower(BLS);
        backRight.setPower(BRS);
    }
    private void wheelTarPos(int FLP, int FRP, int BLP, int BRP){
        frontLeft.setTargetPosition(FLP);
        frontRight.setTargetPosition(FRP);
        backLeft.setTargetPosition(BLP);
        backRight.setTargetPosition(BRP);
    }

    private void forward(double speed, int pos){
        wheelTarPos(pos, pos, pos, pos);
        setModeTarPos();
        setSpeed(speed, speed, speed, speed);
        resetEncoder();
    }
    private void backward(double speed, int pos){
        wheelTarPos(-pos, -pos, -pos, -pos);
        setModeTarPos();
        setSpeed(-speed, -speed, -speed, -speed);
        resetEncoder();
    }

    private void strafeLeft(double speed, int pos){
        wheelTarPos(-pos, pos, pos, -pos);
        setModeTarPos();
        setSpeed(-speed, speed, speed, -speed);
        resetEncoder();
    }
    private void strafeRight(double speed, int pos){
        wheelTarPos(pos, -pos, -pos, pos);
        setModeTarPos();
        setSpeed(speed, -speed, -speed, speed);
        resetEncoder();
    }

    private void turnLeft(double speed, int pos){
        wheelTarPos(-pos, -pos, pos, pos);
        setModeTarPos();
        setSpeed(-speed, -speed, speed, speed);
        resetEncoder();
    }
    private void turnRight(double speed, int pos){
        wheelTarPos(pos, pos, -pos, -pos);
        setModeTarPos();
        setSpeed(speed, speed, -speed, -speed);
        resetEncoder();
    }

    private void lift(double speed, int pos){
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(speed);
        resetEncoder();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setDirection();
        while(opModeIsActive()){
            forward(driveSpeed, countPer10CM);
        }
    }
}
