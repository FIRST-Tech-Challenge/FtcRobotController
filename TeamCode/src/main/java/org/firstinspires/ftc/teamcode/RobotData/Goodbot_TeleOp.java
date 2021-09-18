package org.firstinspires.ftc.teamcode.RobotData;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Good Bot TeleOp ", group = "Opmode")
public class Goodbot_TeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRight;
    private DcMotor rightFront;
    private DcMotor rightRear;

    @Override
    public void runOpMode() throws InterruptedException
    {                   // Naming the motors in configuration
            leftFront = hardwareMap.dcMotor.get("leftFront");
            leftRight = hardwareMap.dcMotor.get("leftRight");
            rightFront = hardwareMap.dcMotor.get("rightFront");
            rightRear = hardwareMap.dcMotor.get("rightRear");

                    //Setting the direction of the motors
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftRight.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightRear.setDirection(DcMotor.Direction.FORWARD);




            waitForStart();

            while(opModeIsActive())
            {
                idle();
            }

    }
}
