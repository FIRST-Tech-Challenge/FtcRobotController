package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="jarlsberg")
public class JarlsbergOpmode extends OpMode {

    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx FourBarRight = null;
    private DcMotorEx FourBarLeft = null;

    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        FourBarRight = hardwareMap.get(DcMotorEx.class, "fourBarRight");
        FourBarLeft = hardwareMap.get(DcMotorEx.class, "fourBarLeft");

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        FourBarLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {

          float x = gamepad1.left_stick_x;
          float y = -gamepad1.left_stick_y;
          float rx = gamepad1.right_stick_x;

          float rT = gamepad1.left_trigger;
          float lT = gamepad1.right_trigger;

          leftFront.setPower(y+x+rx);
          rightFront.setPower(y-x-rx);
          leftBack.setPower(y-x+rx);
          rightBack.setPower(y+x-rx);

          if(FourBarRight.getCurrentPosition() <= 0.1) {
              FourBarRight.setPower(rT);
              FourBarLeft.setPower(rT);
          }

          FourBarRight.setPower(-lT);
          FourBarLeft.setPower(-lT);

    }

}
