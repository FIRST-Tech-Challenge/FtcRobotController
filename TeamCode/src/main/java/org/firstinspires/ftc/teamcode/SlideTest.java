package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Slide Motor Test", group = "TeleOp")
public class SlideTest extends LinearOpMode {


    @Override
    public void runOpMode() {
//        BaseRobot baseRobot = new BaseRobot(hardwareMap, gamepad1, gamepad2, telemetry);
        final DcMotor leftSlide;
        final Gamepad auxGamepad = gamepad2;
        int leftPosition = 0;
        int leftTarget = 0;
        int leftResidual = 0;

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.setAutoClear(false);
        Telemetry.Item leftPos = telemetry.addData("leftSlidePosition", leftPosition);
        Telemetry.Item leftTar = telemetry.addData("leftSlideTarget", leftTarget);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            leftSlide.setTargetPosition(leftTarget);
            leftPosition = leftSlide.getCurrentPosition();
            leftPos.setValue(leftPosition);
            leftTar.setValue(leftTarget);
            telemetry.update();

            if (auxGamepad.dpad_up && leftTarget < 3200) {
                leftTarget += 2;
            } else if (auxGamepad.dpad_down && leftTarget > 0) {
                leftTarget -= 2;
            }
            if (auxGamepad.dpad_right) {
                leftSlide.setPower(0.1);
            } else if (auxGamepad.dpad_left) {
                leftSlide.setPower(-0.1);
            }

            leftResidual = leftPosition - leftTarget;

            if (leftResidual < 10 && leftResidual > -10) {
                leftSlide.setPower(0);
            } else {
                leftSlide.setPower(0.4);
            }
//            } else {
////                leftSlide.setPower(0);
//            }

//            baseRobot.driveGamepads();

//            if (Settings.Deploy.ODOMETRY) {
//                baseRobot.odometry.update();
//            }
        }
//        baseRobot.shutDown();
    }

}