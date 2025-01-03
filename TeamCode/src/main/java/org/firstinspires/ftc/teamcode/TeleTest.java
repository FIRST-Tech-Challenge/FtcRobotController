package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;
import org.firstinspires.ftc.teamcode.hardware.SpecimenGrabber;

@TeleOp(name = "TeleTest")
public class TeleTest extends OpMode {
    private MecanumDrive drive = null;
    private Intake intake = new Intake();  // get the intake class
    private Slide intakeSlide = new Slide("slide", "", Slide.ExtendMotorDirection.Forward, 1300, 1.0, 114.28);
    private Slide clawSlide = new Slide("lift", "resetlift", Slide.ExtendMotorDirection.Reverse, 3192, 1.0,86); //68.568
    private SpecimenGrabber specimanGrabber = new SpecimenGrabber();
    private Telemetry.Item output = null;
    private Telemetry.Item output2 = null;
    Gamepad prevGamepad1 = new Gamepad();
    Gamepad currGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();
    Gamepad currGamepad2 = new Gamepad();
    Intake.IntakePositon desiredPosition = Intake.IntakePositon.HOME;

    public void processSpecimanGrabber() {
        if(currGamepad2.x && !prevGamepad2.x) {
            specimanGrabber.processOpenClose();
        } else if (currGamepad2.y && !prevGamepad2.y){
            clawSlide.MoveTo(44, 1);
        }
    }
    private void processIntake() {
        //hold gp1.right_bumper for sample in
        //hold gp1.left_bumper for sample out
        //release either for sample stop
        //toggle between drive and pickup positions with gp1.y


        //toggle between drive and pickup positions
        if (currGamepad1.y && !prevGamepad1.y) {
            if(desiredPosition == Intake.IntakePositon.DRIVE){
                desiredPosition = Intake.IntakePositon.PICKUP;
            }
            else if(desiredPosition == Intake.IntakePositon.PICKUP || desiredPosition == Intake.IntakePositon.HOME) {
                desiredPosition = Intake.IntakePositon.DRIVE;
            }
        }

        //send intake commands based on desired state
        if (desiredPosition == Intake.IntakePositon.DRIVE){
            intake.goToDrive();
        } else if (desiredPosition == Intake.IntakePositon.PICKUP) {
            intake.goToPickup();
        }

        //in/out/stop of sample
        if (currGamepad1.right_bumper && !currGamepad1.left_bumper) {
            intake.intakeIn();
        }
        else if (!currGamepad1.right_bumper && currGamepad1.left_bumper) {
            intake.intakeOut();
        }
        else if (currGamepad1.right_bumper && currGamepad1.left_bumper) {
            intake.intakeStop();
        }
        else if (!currGamepad1.right_bumper && !currGamepad1.left_bumper) {
            intake.intakeStop();
        }
    }

    private void processSlide() {
        if (currGamepad1.left_trigger == 0 && currGamepad1.right_trigger == 0) {
            intakeSlide.Stop();
        }
        else if (currGamepad1.left_trigger != 0 && currGamepad1.right_trigger == 0){
            intakeSlide.Retract(currGamepad1.left_trigger);
        }
        else if (currGamepad1.right_trigger != 0 && currGamepad1.left_trigger == 0) {
            intakeSlide.Extend(currGamepad1.right_trigger);
        }
        else if(currGamepad1.left_trigger > 0 && currGamepad1.right_trigger > 0) {
            intakeSlide.Stop();
        }
    }


public void processClawStop() {
    if (currGamepad2.dpad_right && !prevGamepad2.dpad_right) {
        clawSlide.Stop();
    }
}
public void processClawManualDown() {
    if(currGamepad2.left_trigger > 0.1) {
        clawSlide.Retract(currGamepad1.left_trigger);
    }
}

public void processLiftDPad() {
    if (currGamepad2.dpad_up && !prevGamepad2.dpad_up) {
        clawSlide.MoveTo(16.5,1);
    }
    else if (currGamepad2.dpad_down && !prevGamepad2.dpad_down) {
        //hang specimen
        hangSpecimen();

    }
}
    @Override
    public void init() {
        prevGamepad1.copy(gamepad1);
        currGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);
        clawSlide.Init(hardwareMap);
        output = telemetry.addData("ticks", clawSlide.motor.getCurrentPosition());
        currGamepad2.copy(gamepad2);
        specimanGrabber.Init(hardwareMap);
        intake.Init(hardwareMap);
        intakeSlide.Init(hardwareMap);
        specimanGrabber.Init(hardwareMap);
        clawSlide.Init(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }



    @Override
    public void loop() {
        currGamepad1.copy(gamepad1);
        currGamepad2.copy(gamepad2);
        output.addData("ticks", clawSlide.motor.getCurrentPosition());
        drive.setPowersFeildCentric(new PoseVelocity2d(
                new Vector2d(
                        currGamepad1.left_stick_x,
                        -currGamepad1.left_stick_y
                ),
                currGamepad1.right_stick_x
        ), 1.0);
        processIntake();
        processSlide();
        processClawStop();
        processLiftDPad();
        processClawManualDown();
        processSpecimanGrabber();
        prevGamepad1.copy(currGamepad1);
        prevGamepad2.copy(currGamepad2);

    }
    //###################ACTIONS #####################
    //BEGIN ACTIONS
    public void hangSpecimen() {
        try {
            clawSlide.MoveTo(0, 0.75);
            Thread.sleep(250);
            specimanGrabber.Open();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }}
    //END ACTIONS





