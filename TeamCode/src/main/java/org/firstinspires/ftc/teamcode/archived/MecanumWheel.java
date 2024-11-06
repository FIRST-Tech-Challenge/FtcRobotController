package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
public class MecanumWheel extends OpMode {
    DcMotor RFmotor;
    DcMotor LFmotor;
   //DcMotor RBmotor;
    DcMotor LBmotor;

    @Override
    public void init() {
        RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
        LFmotor = hardwareMap.get(DcMotor.class, "LFmotor");
     //  RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");
        LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");


        RFmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // RBmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        drivetrainTeleOp();
    }

    public void drivetrainTeleOp() {
        double northsouth = -gamepad1.left_stick_y;
        double westeast = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        RFmotor.setPower(pivot + (-northsouth + westeast));
        //RBmotor.setPower(pivot + (-northsouth - westeast));
        LFmotor.setPower(pivot + (-northsouth - westeast));
        LBmotor.setPower(pivot + (-northsouth + westeast));

        if (gamepad1.y) {
            RFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
          // RBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            LFmotor.setDirection(DcMotorSimple.Direction.FORWARD);
            LBmotor.setDirection(DcMotorSimple.Direction.FORWARD);

         if (gamepad1.b) {
             RFmotor.setDirection(DcMotorSimple.Direction.FORWARD);
             LFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
            // RBmotor.setDirection(DcMotorSimple.Direction.FORWARD);
             LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);


        }
    }
}}