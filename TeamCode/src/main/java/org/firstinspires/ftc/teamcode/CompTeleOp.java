package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FinalDrive", group = "Taus")
//@Disabled
public class CompTeleOp extends LinearOpMode{

    int START_TICKS = 271;
    int INTAKE_TICKS = 0;
    int LOW_TICKS = 2063;
    int MID_TICKS = 3500;
    int HIGH_TICKS = 4900;


    private Hardware robot = new Hardware(false);
    private DrivebaseMethods drive = new DrivebaseMethods(robot);
    private ManipulatorMethods manipulator = new ManipulatorMethods(robot);

    public void runOpMode() {
        robot.initializeHardware(hardwareMap);

        waitForStart();

        telemetry.addLine("Running");

        while (opModeIsActive()) {
            //DRIVEBASE
            if (gamepad1.right_trigger > 0.5) {
                drive.move(gamepad1, 0.3);
            } else {
                drive.move(gamepad1);
            }

            /***********************************************************************************************************************************/
            //INTAKE
            if (gamepad1.right_bumper) {
                manipulator.intake();
            } else if (gamepad1.left_bumper) {
                manipulator.outtake();
            } else {
                manipulator.stopIntake();
            }

            /***********************************************************************************************************************************/
            //SLIDES
            if(gamepad2.x) {
                manipulator.resetSlides();
            }
            if (gamepad2.dpad_up) {
                manipulator.moveSlides(0.5);
            } else if (gamepad2.dpad_down) {
                manipulator.moveSlides(-0.5);
            } else {
                if (gamepad1.y) {
                    manipulator.moveSlideEncoder(START_TICKS,0.5);
                } else if (gamepad1.a) {
                    manipulator.moveSlideEncoder(INTAKE_TICKS,0.5);
                } else if (gamepad2.a) {
                    manipulator.moveSlideEncoder(LOW_TICKS,0.8);
                } else if (gamepad2.b) {
                    manipulator.moveSlideEncoder(MID_TICKS,0.8);
                } else if (gamepad2.y) {
                    manipulator.moveSlideEncoder(HIGH_TICKS,0.8);
                } else if (robot.rightSlides.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)){
                    manipulator.moveSlideEncoder(robot.leftSlides.getCurrentPosition(), robot.rightSlides.getCurrentPosition(), 0.5);
                }
            }

            telemetry.addData("Right Slides Encoder: ", robot.rightSlides.getCurrentPosition());
            telemetry.addData("Left Slides Encoder: ", robot.leftSlides.getCurrentPosition());
            telemetry.update();
        }
    }
}
