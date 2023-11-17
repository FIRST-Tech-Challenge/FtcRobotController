package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mechanism Test", group = "Teleop")
public class MechanismTest extends OpMode
{
    //intake declaration
    public CRServo counteroller;
    public DcMotor intakeMotor;


    //deposit servo declaration
    public Servo depositServoOne;
    public Servo depositServoTwo;


    //servo variable declarations
    boolean leftUp = false;
    boolean rightUp = false;

    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    double  position = MIN_POS;


    @Override
    public void init() //initialization method
    {
        /*
        //intake servo config
        counteroller = hardwareMap.get(CRServo.class, "Intake Servo");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
*/

        //Deposit servo config
        depositServoOne = hardwareMap.get(Servo.class, "Right Deposit");
        depositServoTwo = hardwareMap.get(Servo.class, "Left Deposit");

    }

    @Override
    public void loop() //teleop loop
    {
        /*
        //intake
        if (gamepad1.right_trigger > 0.05) {
            counteroller.setPower(1);
            intakeMotor.setPower(-gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.05){
            counteroller.setPower(-1);
            intakeMotor.setPower(gamepad1.left_trigger);
        } else {
            counteroller.setPower(0);
            intakeMotor.setPower(0);
        }
*/
//        deposit
//right trigger
        if (gamepad2.right_trigger > 0.05) {
            depositServoOne.setPosition(1);
        } else{
            depositServoOne.setPosition(0);
        }
        if (gamepad2.left_trigger > 0.05) {
            depositServoTwo.setPosition(1);
        } else {
            depositServoTwo.setPosition(0);
        }
    }
}
