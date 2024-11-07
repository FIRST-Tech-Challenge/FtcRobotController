package org.firstinspires.ftc.teamcode;


import com.fasterxml.jackson.databind.ser.Serializers;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// TODO: FINISH THIS (I need to read more of the codebase to understand how to integrate this)
// In the meantime, use "SlideTest.java" for a working version that isn't integrated with the rest of the 

public class LinearActuatorBeta {

    public final DcMotor leftSlide;
    private final HardwareMap hardwareMap;
    private final double speed = 0.4;
    private final double maxHeight = 3200;
    private final BaseRobot baseRobot;
    public final int tolerance = 20;
    private int leftPosition = 0;
    private int leftTarget = 0;
    private int leftResidual = 0;

    public LinearActuatorBeta(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE); // Makes encoder values positive
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Resets motor encoder
        leftSlide.setTargetPosition(0); // Sets motor position to allow run to position init
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // using run to position to allow resistance to change
    }

    public void changePosition(int position) {
        leftSlide.setTargetPosition(leftTarget);

        telemetry.setAutoClear(false);
        Telemetry.Item leftPos = telemetry.addData("leftSlidePosition", leftPosition);
        Telemetry.Item leftTar = telemetry.addData("leftSlideTarget", leftTarget);
        telemetry.update();


        leftPosition = leftSlide.getCurrentPosition();

        leftPos.setValue(leftPosition);
        leftTar.setValue(leftTarget);
        telemetry.update();

        if (auxGamepad.dpad_up && leftTarget < maxHeight) {
            leftTarget += 2;
        } else if (auxGamepad.dpad_down && leftTarget > 0) {
            leftTarget -= 2;
        }

        leftResidual = Math.abs(leftPosition - leftTarget);

        if (leftResidual < tolerance && leftResidual > -1 * tolerance) {
            leftSlide.setPower(0);
        } else {
            leftSlide.setPower(speed);
        }
    }

}