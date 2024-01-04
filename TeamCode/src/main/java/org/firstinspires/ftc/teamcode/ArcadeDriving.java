package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ArcadeDriving")

public class ArcadeDriving extends LinearOpMode {
    public DcMotor back_Left;
    public DcMotor back_Right;
    public DcMotor front_Left;
    public DcMotor front_Right;
    public Blinker expansion_Hub_2;
    public Blinker expansion_Hub_9;
    float front_LeftPower, back_LeftPower, front_RightPower, back_RightPower, drive, strafe, spin;
    @Override
    public void runOpMode() {
            back_Left = hardwareMap.get(DcMotor.class, "back_Left");
            back_Right = hardwareMap.get(DcMotor.class, "back_Right");
            front_Left = hardwareMap.get(DcMotor.class, "front_Left");
            front_Right = hardwareMap.get(DcMotor.class, "front_Right");
            expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
            expansion_Hub_9 = hardwareMap.get(Blinker.class, "Expansion Hub 9");

            //front_Left.setDirection(DcMotor.Direction.REVERSE);
            //back_Left.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData("Mode", "waiting");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {

                drive = gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                spin = gamepad1.right_stick_x;

                //see if one or the other works better than the other or are the same
                front_LeftPower = drive + strafe + spin;
                back_LeftPower = drive - strafe - spin;
                front_RightPower = drive - strafe + spin;
                back_RightPower = drive + strafe - spin;

                //front_Left.setPower(drive + strafe + spin);
                //back_Left.setPower(drive - strafe - spin);
                //front_Right.setPower(drive - strafe + spin);
                //back_Right.setPower(drive + strafe - spin);

                front_Left.setPower(Range.clip(front_LeftPower, -1.0, 1.0));
                back_Left.setPower(Range.clip(back_LeftPower, -1.0, 1.0));
                front_Right.setPower(Range.clip(front_RightPower, -1.0, 1.0));
                back_Right.setPower(Range.clip(back_LeftPower, -1.0, 1.0));

                /*

                telemetry.addData("Mode", "running");
                telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
                telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
                telemetry.update();

                idle();

                 */
            }
        }

    }
