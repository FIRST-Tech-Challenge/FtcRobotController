package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.tools.SetDriveMotors;
import org.firstinspires.ftc.teamcode.tools.RobotStateMachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "FC_square_TO_drive", group = "Testing")
public class TeleopDrive extends LinearOpMode {
    private SetDriveMotors setDriveMotorsObj;

    private SlideLift slideLiftObj;
    private Intake intakeObj;
    private Hanger hangerObj;
    private Button handlerButtonA;
    private Button handlerButtonB;
    RobotStateMachine stateMachine;

    public void Setup(){
        setDriveMotorsObj = new SetDriveMotors(hardwareMap, gamepad1);
        //private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //TouchSensor touchDown = hardwareMap.touchSensor.get("touchDown");

       /* slideLiftObj = new SlideLift(hardwareMap);
        intakeObj = new Intake(hardwareMap);
        hangerObj = new Hanger(hardwareMap);*/

        handlerButtonA = new Button(gamepad2, Button.NAME.A);
        handlerButtonB = new Button(gamepad2, Button.NAME.B);
        stateMachine = new RobotStateMachine(hardwareMap, handlerButtonA, handlerButtonB);
    }

    public boolean atRest(){
        return(
                Math.abs(gamepad1.left_stick_y) < setDriveMotorsObj.DEADZONE_MIN_Y &&
                        Math.abs(gamepad1.right_stick_y) < setDriveMotorsObj.DEADZONE_MIN_Y &&
                        Math.abs(gamepad1.left_stick_x) < setDriveMotorsObj.DEADZONE_MIN_X &&
                        Math.abs(gamepad1.right_stick_x) < setDriveMotorsObj.DEADZONE_MIN_X);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();
        while(opModeIsActive()){
//            if(drive.isBusy()&& atRest()){
//                drive.update();
//            }
            telemetry.update();
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean goFast = gamepad1.left_bumper;
//            if (!drive.isBusy() || !atRest()) {
//                setMotorsObj.driveCommands(hardwareMap, horizontal, vertical, turn, goFast);
//            }
            setDriveMotorsObj.driveCommands(horizontal, vertical, turn, goFast);

            //slideLiftObj.setLiftPower(gamepad2.left_stick_y, touchDown); //not ready to test yet

            /*intakeObj.setIntakePower(driverButtonA);
            hangerObj.setHangerPower(driverButtonB);*/
            updateButtons();
            stateMachine.update();
            telemetry.addLine("Is handler button A pressed?" + handlerButtonA.Pressed());
        }
    }
    private void updateButtons(){
        handlerButtonA.updateButton(gamepad2);
        handlerButtonB.updateButton(gamepad2);
    }
}