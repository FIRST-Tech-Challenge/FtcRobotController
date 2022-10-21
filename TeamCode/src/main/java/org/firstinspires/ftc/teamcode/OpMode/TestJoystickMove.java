package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.functions.Arm;
import org.firstinspires.ftc.teamcode.functions.MoveJoystick;
import org.firstinspires.ftc.teamcode.functions.RotationDetector;
import org.firstinspires.ftc.teamcode.functions.Vacuum;
import org.firstinspires.ftc.teamcode.functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;

@TeleOp(name="TestJoystickMove", group="TEST")
public class TestJoystickMove extends OpMode {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, armMotorRight,armMotorLeft, vaccumMotor;
    private Move move;
    private Rotate rotate;
    private Arm arm;
    private Vacuum vaccum;
    private RotationDetector rotationDetector;
    private MoveJoystick moveJoystick;

    private Collector collector, collector2;
    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        vaccumMotor = hardwareMap.dcMotor.get("VM");
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        arm = new Arm(armMotorLeft, armMotorRight);
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        vaccum = new Vacuum(vaccumMotor);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        moveJoystick = new MoveJoystick(rotationDetector, move, rotate);

    }
    boolean k1 = false;
    boolean k2 = false;
    boolean k3 = false;
    boolean k4 = false;
    boolean fata = false;
    boolean slide = false;
    @Override
    public void loop() {

        if(gamepad1.dpad_up) //merge in fata
        {
            move.MoveFull(1);

        }
        else if(gamepad1.dpad_down) //merge in spate
        {
            move.MoveFull(2);
        }
        else if(gamepad1.dpad_left) //merge stanga
        {
            rotate.RotateFull(1);

        }
        else if(gamepad1.dpad_right) // merge dreapta
        {
            rotate.RotateFull(2);
        }
        //miscarea stanga-dreapta:

        //cadranul 1
        if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_y > 0){
            k2 = false;
            k3 = false;
            k4 = false;
            fata=false;
            slide = false;

            double tg = gamepad1.right_stick_y / gamepad1.right_stick_y;
            double angle = Math.atan(tg);
            int angleGrade = 360 - (int) Math.toDegrees(angle);
            //int angleGrade = 315;
            if (k1 == false) {
                moveJoystick.RotateWithJoystick(angleGrade);
            }
            k1 = true;
            move.MoveRaw(1,1);
        }

        if (gamepad1.right_stick_x < 0 && gamepad1.right_stick_y > 0){
            k1 = false;
            k3 = false;
            k4 = false;
            fata=false;
            slide = false;

            double tg = gamepad1.right_stick_y / gamepad1.right_stick_y;
            double angle = Math.atan(tg);
            int angleGrade = (int) Math.toDegrees(angle);
            //int angleGrade = 315;
            if (k2 == false) {
                moveJoystick.RotateWithJoystick(angleGrade);
            }
            k2 = true;
            move.MoveRaw(1,1);
        }
        // cadranul 3
        if (gamepad1.right_stick_x < 0 && gamepad1.right_stick_y < 0){
            k1 = false;
            k2 = false;
            k4 = false;
            fata=false;
            slide = false;

            double tg = gamepad1.right_stick_y / gamepad1.right_stick_y;
            double angle = Math.atan(tg);
            int angleGrade = (int) Math.toDegrees(angle);
            //int angleGrade = 315;
            if (k3 == false) {
                moveJoystick.RotateWithJoystick(angleGrade);
            }
            k3 = true;
            move.MoveRaw(1,1);
        }

        //cadranul 4
        if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_y < 0){
            k1 = false;
            k2 = false;
            k3 = false;
            fata=false;
            slide = false;

            double tg = gamepad1.right_stick_y / gamepad1.right_stick_y;
            double angle = Math.atan(tg);
            int angleGrade = (int) Math.toDegrees(angle);
            //int angleGrade = 315;
            if (k4 == false) {
                moveJoystick.RotateWithJoystick(angleGrade);
            }
            k4 = true;
            move.MoveRaw(1,1);
        }

        //merge fata
        if (gamepad1.right_stick_y > 0 && gamepad1.right_stick_x == 0){
            k1 = false;
            k2 = false;
            k3 = false;
            k4 = false;
            slide = false;
            int angleGrade = 0;
            //int angleGrade = 315;
            if(fata == false) {
                moveJoystick.RotateWithJoystick(angleGrade);
            }
            fata = true;
            move.MoveRaw(1,1);
        }

        //merge spate
        if (gamepad1.right_stick_y < 0 && gamepad1.right_stick_x == 0){
            k1 = false;
            k2 = false;
            k3 = false;
            k4 = false;
            slide = false;
            int angleGrade = 0;
            //int angleGrade = 315;
            if(fata == false) {
                moveJoystick.RotateWithJoystick(angleGrade);
            }
            fata = true;
            move.MoveRaw(2,1);
        }
        //merge stanga
        if (gamepad1.right_stick_y == 0 && gamepad1.right_stick_x < 0){
            k1 = false;
            k2 = false;
            k3 = false;
            k4 = false;
            fata = false;
            int angleGrade = 0;
            //int angleGrade = 315;
            if(slide == false) {
                moveJoystick.RotateWithJoystick(angleGrade);
            }
            slide = true;
            move.MoveRaw(3,1);
        }
        //merge dreapta
        if (gamepad1.right_stick_y == 0 && gamepad1.right_stick_x > 0){
            k1 = false;
            k2 = false;
            k3 = false;
            k4 = false;
            fata = false;
            int angleGrade = 0;
            //int angleGrade = 315;
            if(slide == false) {
                moveJoystick.RotateWithJoystick(angleGrade);
            }
            slide = true;
            move.MoveRaw(4,1);
        }



        if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0){
            k1 = false;
            k2 = false;
            k3 = false;
            k4 = false;
            slide = false;
            fata = false;
        }

        if(gamepad1.b){
            vaccum.SwitchAndWait(1, getRuntime());
        }
        if(gamepad1.x)
        {
            vaccum.SwitchAndWaitInv(1,getRuntime());
        }
        telemetry.update();
    }
}

