package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Swicher")
public class EasyerDriveSwich extends BasicOpMode_Linear{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    boolean toggle = false;

    @Override
    public void  runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();
        resetRuntime();

        while (opModeIsActive()){
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = 0;
            double rightFrontPower = 0;
            double leftBackPower = 0;
            double rightBackPower = 0;

           if (Math.abs(axial) > 0.05 || Math.abs(lateral) > 0.05 || Math.abs(yaw) > 0.05) {
               leftFrontPower =  axial + lateral + yaw;
               rightFrontPower = axial - lateral - yaw;
               leftBackPower = axial - lateral + yaw;
               rightBackPower = axial + lateral - yaw;
           }

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.x) {
                toggle = !toggle;
                sleep(50);
            }


          //  if (gamepad1.left_stick_y > 0.05) {
                if (toggle) {
                    leftFrontDrive.setPower(leftFrontPower / 2);
                    rightFrontDrive.setPower(rightFrontPower / 2);
                    leftBackDrive.setPower(leftBackPower / 2);
                    rightBackDrive.setPower(rightBackPower / 2);
                } else {
                    leftFrontDrive.setPower(leftFrontPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    leftBackDrive.setPower(leftBackPower);
                    rightBackDrive.setPower(rightBackPower);
                }
            }

        }


    private void drive(float speed){

            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);


    }
}
