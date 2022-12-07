package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ArmServos;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.Vacuum;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;

@Disabled
@TeleOp(name="Test David", group="TEST")
public class TestDavid extends OpMode {


    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack,vaccumLeft, carouselMotor;
    private DcMotorEx armMotorLeft, armMotorRight;
    //private Servo collectorServo;
    private Servo level1, level2;
    private CRServo collectorCr;
    private Move move;
    private Rotate rotate;
    private Arm arm;
    private Vacuum vaccum;
    private RotationDetector rotationDetector;
    private ArmEncoder armEncoder;
    //private CarouselMotor _carouselMotor;
    //private VaccumServo vaccumServo;
    private Collector collector;

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
        //arm = new Arm(armMotorLeft, armMotorRight);
      //  armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
       // vaccum = new Vacuum(vaccumLeft);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        //moveJoystick = new MoveJoystick(rotationDetector, move, rotate);
        //_carouselMotor = new CarouselMotor(carouselMotor);
        collectorCr = hardwareMap.crservo.get("CR");
        level1 = hardwareMap.servo.get("L1");
        level2 = hardwareMap.servo.get("L2");
        armServos = new ArmServos(level1, level2);
        collector = new Collector(collectorCr);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {



        if(gamepad1.dpad_up) //merge in fata
        {
            move.MoveFull(2);

        }
        else if(gamepad1.dpad_down) //merge in spate
        {
            move.MoveFull(1);
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
            move.MoveRaw(3,0.5);
        }
        else if(gamepad1.left_bumper)
        {
            move.MoveRaw(4,0.5);
        }
        if(!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.right_bumper && !gamepad1.left_bumper
                && gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x==0 && !gamepad1.a && !gamepad1.b && gamepad1.right_stick_y==0)
        {
            move.StopSlow();
        }
        if(gamepad2.right_stick_y>0)
        {
            arm.StartInvers(gamepad2.right_stick_y/2);
        }
        if(gamepad2.right_stick_y<0)
        {
            arm.Start(gamepad2.right_stick_y/2);
        }
        else if(gamepad2.right_stick_y==0) arm.Stop();
//        if(gamepad2.a)
//        {

//            armEncoder.goTo(120,115);
//        }
        if(gamepad2.left_bumper) vaccumLeft.setPower(0.5);
        if(gamepad2.right_bumper) vaccumLeft.setPower(-0.5);
         if(gamepad2.x)
         {
             collector.SwitchAndWaitContinuous(1, getRuntime());
         }
         telemetry.update();
         if(gamepad2.b)
         {
             armServos.SwitchAndWaitLevel2(1, getRuntime());
         }
         if(gamepad2.a)
         {
             armServos.SwitchAndWaitLevel1(1,getRuntime());
         }

    }
}
