package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MechanismTestTeleOp", group = "Teleops")
@Config
public class TestMechanismTeleOp extends OpMode{

    public static boolean[] motorPorts = new boolean[4];
    public static boolean[] servoPorts = new boolean[6];

    int curMotor = 0;
    int curServo = 0;

    ElapsedTime clickTime = new ElapsedTime();

    DcMotor[] motors = new DcMotor[4];
    Servo[] servos = new Servo[6];

    @Override
    public void init() {
        checkInit();
        int index = 0;
        for (DcMotor motor : motors)
        {
            if (motor != null)
            {
                curMotor = index;
            }
            index++;
        }
        index = 0;
        for (Servo servo : servos)
        {
            if (servo != null)
            {
                curServo = index;
            }
            index++;
        }
    }

    void checkInit()
    {
        int index = 0;
        for (boolean port : motorPorts)
        {
            if (port)
            {
                motors[index] = hardwareMap.dcMotor.get("motor" + index);
                return;
            }
            index++;
        }

        index = 0;
        for (boolean port : servoPorts)
        {
            if (port)
            {
                servos[index] = hardwareMap.servo.get("servo" + index);
                return;
            }
            index++;
        }
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_right && clickTime.milliseconds() > 500)
        {
            clickTime.reset();
            for (int i = curMotor + 1; i < motors.length + curMotor; i++)
            {
                if (motors[i % motors.length] != null) {
                    curMotor = i % motors.length;
                    break;
                }
            }
        }
        else if (gamepad1.dpad_left && clickTime.milliseconds() > 500)
        {
            clickTime.reset();
            curMotor--;
            if (curMotor < 0)
                curMotor = 3;
        }
        else if (gamepad1.dpad_up && clickTime.milliseconds() > 500)
        {
            clickTime.reset();
            curServo++;
            if (curServo > 5)
                curServo = 0;
        }
        else if (gamepad1.dpad_down && clickTime.milliseconds() > 500)
        {
            clickTime.reset();
            curServo--;
            if (curServo < 0)
                curServo = 5;
        }

        if (gamepad1.a)
        {
            motors[curMotor].setPower(1);
            servos[curServo].setPosition(1);
        }
        else if (gamepad1.b)
        {
            motors[curMotor].setPower(-1);
            servos[curServo].setPosition(0);
        }
        else
        {
            motors[curMotor].setPower(0);
        }
    }
}
