package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MechanismTestTeleOp", group = "Teleops")
@Config
public class TestMechanismTeleOp extends OpMode{

    public static boolean[] motorPorts = new boolean[4];
    public static boolean[] servoPorts = new boolean[6];

    DcMotor[] motors = new DcMotor[4];
    Servo[] servos = new Servo[6];

    @Override
    public void init() {
        checkInit();
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
        if (gamepad1.a)
        {
            for (DcMotor motor : motors)
            {
                if (motor != null)
                {
                    motor.setPower(1);
                }
            }
            for (Servo servo : servos)
            {
                if (servo != null)
                {
                    servo.setPosition(1);
                }
            }
        }
        else if (gamepad1.b)
        {
            for (DcMotor motor : motors)
            {
                if (motor != null)
                {
                    motor.setPower(-1);
                }
            }
            for (Servo servo : servos)
            {
                if (servo != null)
                {
                    servo.setPosition(0);
                }
            }
        }
        else
        {
            for (DcMotor motor : motors)
            {
                if (motor != null)
                {
                    motor.setPower(0);
                }
            }
        }
    }
}
