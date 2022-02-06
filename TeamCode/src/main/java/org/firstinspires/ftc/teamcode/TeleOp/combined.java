package org.firstinspires.ftc.teamcode.TeleOp;

public class combined extends initializer{

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
