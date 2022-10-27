package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * This file creates a Tank Drive Mode for the Robot
 * In Tank Drive, the left joystick controls the left wheel, and the right
 * joystick controls the right wheel
 */

@TeleOp
@Disabled
public class TankDriveWithExponentiation extends LinearOpMode {

    private DcMotor Motor1 = null;
    private DcMotor Motor2 = null;
    private DcMotor Motor3 = null;
    private DcMotor Motor4 = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Init");
        telemetry.update();

        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");

        Motor1.setDirection(DcMotor.Direction.FORWARD);
        Motor2.setDirection(DcMotor.Direction.FORWARD);
        Motor3.setDirection(DcMotor.Direction.REVERSE);
        Motor4.setDirection(DcMotor.Direction.REVERSE);

//        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double Motor1Power = 0.0;
        double Motor2Power = 0.0;
        double Motor3Power = 0.0;
        double Motor4Power = 0.0;
        waitForStart();

        while(opModeIsActive()){
            double leftJoystick = gamepad1.left_stick_y;
            double rightJoystick = gamepad1.right_stick_y;

            Motor1Power = Range.clip(leftJoystick, -1.0, 1.0);
            Motor2Power = Range.clip(leftJoystick, -1.0, 1.0);
            Motor3Power = Range.clip(rightJoystick, -1.0, 1.0);
            Motor4Power = Range.clip(rightJoystick, -1.0, 1.0);

            // Exponentiation for Motor1
            if(Motor1Power < 0.0){
                Motor1Power = Motor1Power * Motor1Power;
                Motor1Power = Motor1Power * -1;
            }
            else{
                Motor1Power = Motor1Power * Motor1Power;
            }

            // Exponentiation for Motor2
            if(Motor2Power < 0.0){
                Motor2Power = Motor2Power * Motor2Power;
                Motor2Power = Motor2Power * -1;
            }
            else{
                Motor2Power = Motor2Power * Motor2Power;
            }

            // Exponentiation for Motor3
            if(Motor3Power < 0.0){
                Motor3Power = Motor3Power * Motor3Power;
                Motor3Power = Motor3Power * -1;
            }
            else{
                Motor3Power = Motor3Power * Motor3Power;
            }

            // Exponentiation for Motor4
            if(Motor4Power < 0.0){
                Motor4Power = Motor4Power * Motor4Power;
                Motor4Power = Motor4Power * -1;
            }
            else{
                Motor4Power = Motor4Power * Motor4Power;
            }

            Motor1.setPower(Motor1Power);
            Motor2.setPower(Motor2Power);
            Motor3.setPower(Motor3Power);
            Motor4.setPower(Motor4Power);

            telemetry.addData("Motor1", "Forward/Backwards Power (%.2f)", Motor1Power);
            telemetry.addData("Motor2", "Forward/Backwards Power (%.2f)", Motor2Power);
            telemetry.addData("Motor3", "Forward/Backwards Power (%.2f)", Motor3Power);
            telemetry.addData("Motor4", "Forward/Backwards Power (%.2f)", Motor4Power);
            telemetry.update();
        }
    }
}
