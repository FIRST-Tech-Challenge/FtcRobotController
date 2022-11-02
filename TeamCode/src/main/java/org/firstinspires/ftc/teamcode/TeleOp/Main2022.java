package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.ArmServos;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.MoveDirectJoystick;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotateMove;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.Vacuum;

@TeleOp(name="MAIN 4", group="GAME")
public class Main2022 extends OpMode {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, armMotorRight,armMotorLeft, vaccumLeft, armMotorChain;
    private Servo level1, level2;

    private Move move;
    private Rotate rotate;
    private Arm arm;
    private Vacuum vaccum;
    private RotationDetector rotationDetector;
    private RotateMove rotateMove;
    private Collector collector;
    private Servo servoSR, servoLeft, servoRight;
    private CRServo collectorCr;
    private MoveDirectJoystick moveDirectJoystick;

    private ArmServos armServos;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        // vaccumRight = hardwareMap.dcMotor.get("VR");
        vaccumLeft = hardwareMap.dcMotor.get("VL");
        // carouselMotor = hardwareMap.dcMotor.get("CM");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "AMR");
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "AML");
        armMotorChain = hardwareMap.get(DcMotor.class, "AMC");
        arm = new Arm(armMotorLeft, armMotorRight, armMotorChain);
        //armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        // vaccum = new Vacuum(vaccumLeft);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        //moveJoystick = new MoveJoystick(rotationDetector, move, rotate);
        //_carouselMotor = new CarouselMotor(carouselMotor);
        collectorCr = hardwareMap.crservo.get("CR");
        collector = new Collector(collectorCr);
        level1 = hardwareMap.servo.get("L1S");
        level2 = hardwareMap.servo.get("L2S");
        armServos = new ArmServos(level1, level2);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        moveDirectJoystick = new MoveDirectJoystick(move, rotationDetector);
    }

    @Override
    public void loop() {

//        if(gamepad1.right_stick_x!=0 || gamepad1.right_stick_y!=0)
//        {
//            moveDirectJoystick.MoveSimple(-gamepad1.right_stick_x, +gamepad1.right_stick_y);
//        }
        if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0){
            if(Math.abs(gamepad1.left_stick_x)>=Math.abs(gamepad1.left_stick_y)){
                rotate.RotateRaw(1, gamepad1.left_stick_x);
            }
//            else{
//                move.MoveRaw(1, gamepad1.left_stick_y);
//            }
        }
        else if(gamepad1.dpad_up) //merge in fata
        {
            //move.MoveFull(2);
            //move.MoveRaw(2,speed);


        }
        else if(gamepad1.dpad_down) //merge in spate
        {

            //move.MoveFull(1);

        }
        else if(gamepad1.dpad_left) //merge stanga
        {
            rotate.RotateFull(2);

        }
        else if(gamepad1.dpad_right) // merge dreapta
        {
            rotate.RotateFull(1);
        }
        //miscarea stanga-dreapta:
        else if(gamepad1.right_bumper)
        {

            move.MoveRaw(4, 0.5);
        }
        else if(gamepad1.left_bumper)
        {
            move.MoveRaw(3, 0.5);


        }
        else{
            move.MoveStop();
        }
        if(gamepad2.right_stick_y>0)
        {
            arm.Start(gamepad2.right_stick_y);
        }
        if(gamepad2.right_stick_y<0)
        {
            arm.Start(gamepad2.right_stick_y);
        }
        else if(gamepad2.right_stick_y==0) arm.Stop();

        if(gamepad2.left_bumper) vaccumLeft.setPower(0.5);
        if(gamepad2.right_bumper) vaccumLeft.setPower(-0.5);
        if(gamepad2.right_trigger!=0)
        {
            //collector.SwitchAndWaitContinuous(1, getRuntime());
            collector.Start();
        }
        else{
            collector.Stop();
        }
        if(gamepad2.b)
        {
            armServos.Level2Up();
        }
        if(gamepad2.a)
        {
            armServos.Level1Up();
        }
        if(gamepad1.right_trigger>0){
            move.MoveRaw(2,gamepad1.right_trigger);
        }
        if(gamepad1.left_trigger>0){
            move.MoveRaw(1,gamepad1.left_trigger);
        }
        if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
            move.MoveStop();
        }


    }
}
