package org.firstinspires.ftc.teamcode.TeleOp.UntestedFunctionsNew;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.functions.driveMethods;
import org.firstinspires.ftc.teamcode.TeleOp.functions.linSlide2;

@Disabled
public class initializer extends LinearOpMode {
    public DcMotor motorFrontLeft; //motors declared
    public DcMotor motorBackLeft ;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    private DcMotor LinSlideMotorR;
    private DcMotor LinSlideMotorL;
    public DcMotor carousel;
    private Servo servoL;
    private Servo servoR;


    private ElapsedTime runtime;
    private int toggle;//toggle for setting height
    final double modeCD = 0.15;//these two values are for putting a cooldown on switching heights, just in case pushing down the button slightly too long would make it switch heights more than 1 time
    double CDtimer = 0;


    public void initializeDriveTrain() {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        carousel = hardwareMap.dcMotor.get("carousel");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveMethods.setMotors(motorFrontLeft,motorBackLeft,motorFrontRight,motorBackRight);
    }

    public void initializeLinSlide(){
        LinSlideMotorL = hardwareMap.dcMotor.get("LinSlideMotor");//hardware
        LinSlideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //linSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LinSlideMotorR.setDirection(DcMotorSimple.Direction.FORWARD);//change it if needed
        linSlide2.setMotors(LinSlideMotorR);
        runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);//gets time, used for PID
        toggle=0;
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
