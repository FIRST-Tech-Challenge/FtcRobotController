package org.firstinspires.ftc.teamcode.TeleOp.UntestedFunctionsNew;

import org.firstinspires.ftc.teamcode.TeleOp.functions.driveMethods;
import org.firstinspires.ftc.teamcode.TeleOp.functions.linSlide2;

public class combined extends initializer {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeDriveTrain();
        initializeLinSlide();

        while(opModeIsActive()) {
            driveMethods.drivetrainAction(gamepad1);
            linSlide2.desiredSlideHeight(gamepad1);
            linSlide2.moveSlide();

        }
    }
}
