package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;
import org.firstinspires.ftc.teamcode.hardware.SpecimenGrabber;

@TeleOp(name = "Intake Test")
public class TeleTest extends OpMode {
    private MecanumDrive drive = null;
    private Intake intake = new Intake();  // get the intake class
    private Slide intakeSlide = new Slide("slide", "", Slide.ExtendMotorDirection.Forward, 1300, 1.0, 114.28);
    private Slide clawSlide = new Slide("lift", "resetlift", Slide.ExtendMotorDirection.Reverse, 2600, 1.0,68.568);
    private SpecimenGrabber specimanGrabber = new SpecimenGrabber();
    Gamepad prevGamepad1 = new Gamepad();
    Gamepad currGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();
    Gamepad currGamepad2 = new Gamepad();
    Intake.IntakePositon desiredPosition = Intake.IntakePositon.HOME;


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

    @Override
    public void init() {
        prevGamepad1.copy(gamepad1);
        currGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);
        currGamepad2.copy(gamepad2);
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
        drive.setPowersFeildCentric(new PoseVelocity2d(
                new Vector2d(
                        currGamepad1.left_stick_x,
                        -currGamepad1.left_stick_y
                ),
                currGamepad1.right_stick_x
        ), 1.0);
        processIntake();
        processSlide();
        prevGamepad1.copy(currGamepad1);
        prevGamepad2.copy(currGamepad2);

    }
    //###################ACTIONS #####################
    //BEGIN ACTIONS
    public void hangSpecimen() {
        try {
            clawSlide.MoveTo(0, 0.75);
            Thread.sleep(200);
            specimanGrabber.Open();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }}
    //END ACTIONS





