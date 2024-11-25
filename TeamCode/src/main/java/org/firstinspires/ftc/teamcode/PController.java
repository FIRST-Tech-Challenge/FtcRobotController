package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="P controller", group="teleop")
public class PController extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.get(DcMotorEx.class, "arm");
        Servo gate = hardwareMap.get(Servo.class, "gate");
        Servo grabberFront = hardwareMap.get(Servo.class, "grabberFront");
        Servo grabberBack = hardwareMap.get(Servo.class, "grabberBack");


        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate.setDirection(Servo.Direction.REVERSE);
        grabberFront.setDirection(Servo.Direction.REVERSE);
        grabberBack.setDirection(Servo.Direction.FORWARD);


        int target = 170;
        int currentPos;
        double armPower = 0;
        double reference = 0;
        double kCos = 0.2;

        int error = 0;
        //final double p = 0.001;
        PIDController controller = new PIDController(0.05, 0, 0);

        GamepadEx playerOne = new GamepadEx(gamepad1);


        gate.setPosition(0);
        double closePos = 0.05;
        grabberFront.setPosition(closePos);
        grabberBack.setPosition(closePos);

        waitForStart();
        while (opModeIsActive()) {
            //currentPos = arm.getCurrentPosition();
            //error = target - currentPos;
            //armPower = controller.calculate(currentPos, target);
            //armPower = 1;
            if (armPower > 1) {
                armPower = 1;
            } else if (armPower < -1) {
                armPower = -1;
            }
            /*if (error > 5) {
                armPower = 0.3;
            } else if (error < -5) {
                armPower = -0.3;
            } else {
                armPower = 0;
            }*/
            //armPower = error * p;


            //FEEDFORWARD
            //520 ticks in 2pi radians
            /*
            target = 80;
            reference = Math.cos(ticksToRadians(target));
            armPower = kCos * reference;
*/
            armPower = playerOne.getLeftY();
            arm.setPower(armPower);

            playerOne.readButtons();

            if (playerOne.wasJustPressed(GamepadKeys.Button.A)) {
                if (gate.getPosition() == 0) {
                    gate.setPosition(0.5);
                } else {
                    gate.setPosition(0);
                }
            }
            //use the grabber
            if (playerOne.wasJustPressed(GamepadKeys.Button.B)) {
                if (round(grabberFront.getPosition(), 2) == closePos) {
                    grabberFront.setPosition(0.5);
                    grabberBack.setPosition(0.5);
                } else {
                    grabberFront.setPosition(closePos);
                    grabberBack.setPosition(closePos);
                }
            }


            //telemetry.addData("cos", reference);
            //telemetry.addData("currentPos", currentPos);
            //telemetry.addData("error", error);
            telemetry.addData("armPower", armPower);
            telemetry.addData("gatePos", gate.getPosition());
            telemetry.addData("front", grabberFront.getPosition());
            telemetry.addData("frontRounded", round(grabberFront.getPosition(), 2));
            telemetry.addData("back", grabberBack.getPosition());
            telemetry.addData("grabberPos", grabberFront.getPosition() == closePos ? "close" : "open");
            telemetry.addData("grabberPosPostRounding", round(grabberFront.getPosition(), 2) == closePos ? "close" : "open");


            telemetry.update();

        }
    }

    public double ticksToRadians(int ticks) {
        return ticks / 82.76;
    }

    public double round(double num, int places) {
        return Math.round(num * Math.pow(10, places)) / Math.pow(10, places);
    }

}
