package org.firstinspires.ftc.teamcode.Tele.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlide;

@TeleOp
public class experimentalLinSlide extends LinearOpMode {

    public DcMotor linSlideMotor;

    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
            linSlideMotor = hardwareMap.dcMotor.get("linSlideMotor");
            linSlide.setLSMotor(linSlideMotor);

        }

    }

}
