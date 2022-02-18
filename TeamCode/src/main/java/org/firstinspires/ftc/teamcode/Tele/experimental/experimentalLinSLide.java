package org.firstinspires.ftc.teamcode.Tele.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tele.untested.linSlide;

@TeleOp
public class experimentalLinSLide extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        DcMotor linSlideMotor = hardwareMap.dcMotor.get("linSlideMotor");

        waitForStart();

        while (opModeIsActive()) {
            linSlide.setLSMotor(linSlideMotor);
        }

    }

}
