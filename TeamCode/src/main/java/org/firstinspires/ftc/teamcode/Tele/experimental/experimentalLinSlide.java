package org.firstinspires.ftc.teamcode.Tele.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlide;

@TeleOp
public class experimentalLinSlide extends LinearOpMode {

    public DcMotor LinSlideMotor;

    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
            LinSlideMotor = hardwareMap.dcMotor.get("LinSlideMotor");
            linSlide.setLSMotor(LinSlideMotor);

        }

    }

}
