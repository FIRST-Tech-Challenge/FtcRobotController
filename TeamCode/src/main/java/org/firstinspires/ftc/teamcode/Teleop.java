package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "A Teleop", group = "Teleop")
public class Teleop extends LinearOpMode
{
    //button logics
    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;

    //servo variable declarations
    boolean leftUp = true;
    boolean rightUp = true;

    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    double  position = MIN_POS;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Hardware robot = new Hardware(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            robot.robotODrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //a button logic
            if (gamepad1.a && !aPressed)
            {
                rightUp = !rightUp;
                leftUp = !leftUp;
                aPressed = true;
            }
            else if (!gamepad1.a)
                aPressed = false;

            //x button logic
            if (gamepad1.x && !xPressed)
            {
                leftUp = !leftUp;
                xPressed = true;
            }
            else if (!gamepad1.x)
                xPressed = false;

            //b button logic
            if (gamepad1.b && !bPressed)
            {
                rightUp = !rightUp;
                bPressed = true;
            }
            else if (!gamepad1.b)
                bPressed = false;

            //x button drops left
            if (leftUp = true)
                robot.depositServoTwo.setPosition(1);
            else if (leftUp = false)
                robot.depositServoTwo.setPosition(0);

            //b button drops right
            if (rightUp = true)
                robot.depositServoOne.setPosition (1);
            else if (rightUp = false)
                robot.depositServoTwo.setPosition (0);
        }
    }
}
