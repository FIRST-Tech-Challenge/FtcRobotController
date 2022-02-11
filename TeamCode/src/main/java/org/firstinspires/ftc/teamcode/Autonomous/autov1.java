package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.MainOpMode.driveAndLinslide;
import org.firstinspires.ftc.teamcode.TeleOp.UntestedFunctionsNew.initializer;
import org.firstinspires.ftc.teamcode.TeleOp.functions.carousel;
import org.firstinspires.ftc.teamcode.TeleOp.functions.dump;


@Autonomous

public class autov1 extends LinearOpMode {

    public DcMotor LinSlideMotor; //motors declared
    public DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    public DcMotor carouMotor;
    public Servo dumpServo;
    public CRServo continServo;

    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH}
    driveAndLinslide.states state = driveAndLinslide.states.LOW;

    private int toggle;//toggle for setting height
    final double modeCD = 0.15;//these two values are for putting a cooldown on switching heights, just in case pushing down the button slightly too long would make it switch heights more than 1 time
    double CDtimer = 0;

    //Encoder positions for each level on linear slide
    final int low = 0;
    final int mid = 1200;
    final int high = 2600;

    final double encodeNum = 537;

    private ElapsedTime runtime;
    double timeStart = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        LinSlideMotor = hardwareMap.dcMotor.get("LinSlideMotor");

        carouMotor = hardwareMap.dcMotor.get("carouMotor");
        carousel.setCarouMotor(carouMotor);

        continServo = hardwareMap.crservo.get("continServo");

        dumpServo= hardwareMap.servo.get("dumpServo");
        dump.setServo(dumpServo);

        waitForStart();

        if (isStopRequested()) return;

        initialize();
        DTreset();

        planA();




    }


    public void initialize(){//initialize linearSlide and drive motors. it assumes the linear slide starts at the lowest state.
        LinSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);//change it if needed
        //LinSlideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        dump.startPos();

        runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);//gets time
        toggle=0;


    }

    public void DTreset(){
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void planA(){
        turn90(1);
        turn90(1);

        setDist(round(1.2*encodeNum),round(-1.2*encodeNum),round(-1.2*encodeNum),round(1.2*encodeNum));

        timeStart=runtime.time();
        carouMotor.setPower(1);
        while(!(runtime.time()-timeStart>=3)){

        }
        carouMotor.setPower(0);

        turn90(-1);

        setDist(round(encodeNum*0.25),round(encodeNum*-0.25),round(encodeNum*-0.25),round(encodeNum*0.25));

        setDist(round(encodeNum*4.2),round(encodeNum*4.2),round(encodeNum*4.2),round(encodeNum*4.2));



    }

    public void planB(){

    }

    public void turn90(int dir){

        int posEncNum = (int) Math.round(dir*encodeNum * 0.619);
        int negEncNum =(int) Math.round(dir*-encodeNum * 0.619);

        motorFrontLeft.setTargetPosition(posEncNum);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(0.7);

        motorFrontRight.setTargetPosition(negEncNum);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(0.7);

        motorBackLeft.setTargetPosition(posEncNum);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(0.7);

        motorBackRight.setTargetPosition(negEncNum);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(0.7);

        while(motorBackRight.isBusy()||motorBackLeft.isBusy()||motorFrontLeft.isBusy()||motorFrontRight.isBusy()){

        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setDist(int FL,int FR,int BL,int BR){
        motorFrontLeft.setTargetPosition(FL);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(0.7);

        motorFrontRight.setTargetPosition(FR);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(0.7);

        motorBackLeft.setTargetPosition(BL);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(0.7);

        motorBackRight.setTargetPosition(BR);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(0.7);

        while(motorBackRight.isBusy()||motorBackLeft.isBusy()||motorFrontLeft.isBusy()||motorFrontRight.isBusy()){

        }
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int round(double num){
        int result = (int)Math.round(num);
        return result;
    }



}
