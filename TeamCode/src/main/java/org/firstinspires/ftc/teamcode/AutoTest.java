package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.libraries.BasicRobot;
import org.firstinspires.ftc.teamcode.libraries.DriveMainAuto;

@Autonomous
public class AutoTest extends LinearOpMode implements DriveMainAuto, BasicRobot {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        //load motors things
        loadMotors(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        loadAll(hardwareMap);
        outtakeAngle.set(TRANSFER);
        outtakeClaw.set(CLOSE);
        intakeAngle.set(TRANSFER);
        intakeClaw.set(OPEN);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //send telemetry data and wait for start
        DcMotorEx sideways1 = hardwareMap.get(DcMotorEx.class, "sideways");


        telemetry.update();
        waitForStart();
        runtime.reset();

        boolean forward = true;
        boolean where = false;
        double pos ;
        int straightFacing = 180;

        elevatorMove(HIGH_BASKET);
        backward(20, straightFacing);
        straightFacing=straightFacing+45;
        turnTo(straightFacing);
        sidwaysMovement(3, 1, straightFacing);
        // backward(3, straightFacing);


        outtakeAngle.set(PLACE);
        waitMe(0.4);
        outtakeClaw.set(OPEN);
        waitMe(0.4);
        outtakeAngle.set(TRANSFER);
        waitMe(0.4);
        elevatorMove(-HIGH_BASKET);
        straightFacing=straightFacing+45;
        turnTo(straightFacing);

        sidwaysMovement(4, 1, straightFacing);
        intakeAngle.set(GRAB);

        straightFacing = straightFacing + 7;
        turnTo(straightFacing);

        // elevatorMove(300);
        // forward(14, straightFacing);
        // intakeClaw.set(CLOSE);
        // waitMe(0.4);
        // intakeAngle.set(TRANSFER);
        // waitMe(0.8);
        // elevatorMove(-400);
        // elevatorWait();

        // outtakeClaw.set(CLOSE);
        // waitMe(0.4);
        // intakeClaw.set(OPEN);
        // elevatorMove(HIGH_BASKET);
        // backward(10, straightFacing);
        // straightFacing=straightFacing-45;
        // turnTo(straightFacing);
        // waitMe(0.4);

        outtakeAngle.set(PLACE);
        waitMe(0.4);
        outtakeClaw.set(OPEN);
        waitMe(0.4);
        outtakeAngle.set(TRANSFER);
        waitMe(0.4);
        elevatorMove(-HIGH_BASKET);
        straightFacing=straightFacing+45;
        turnTo(straightFacing);

        // sidwaysMovement(-9, -1, straightFacing);

        // elevatorMove(300);
        // forward(14, straightFacing);
        // intakeClaw.set(CLOSE);
        // waitMe(0.4);
        // intakeAngle.set(TRANSFER);
        // waitMe(0.8);
        // elevator(-300);

        // outtakeClaw.set(CLOSE);
        // waitMe(0.4);
        // intakeClaw.set(OPEN);
        // backward(12, straightFacing);
        // sidwaysMovement(9, 1, straightFacing);

        // elevatorMove(HIGH_BASKET);
        // straightFacing=straightFacing-45;
        // turnTo(straightFacing);

        // outtakeAngle.set(PLACE);
        // waitMe(0.4);
        // outtakeClaw.set(OPEN);
        // waitMe(0.4);
        // outtakeAngle.set(TRANSFER);
        // waitMe(0.4);
        // elevatorMove(-HIGH_BASKET);
        // straightFacing=straightFacing+45;
        // turnTo(straightFacing);





        // movementStraight(-14, -1, straightFacing);
        // straightFacing=180+45;
        // runtime.reset();
        // while(imu.notFacing(straightFacing, runtime)){
        //     telemetry.addData("here","no");
        //     telemetry.update();
        //     moveWithCorrection(0.0,straightFacing);
        // }
        // telemetry.update();
        // elavator1.move(3);
        // elavator2.move(3);
        // while(elavator1.getMotor().isBusy()){
        //     telemetry.addData("tar", elavator1.getMotor().getTargetPosition());
        //     elavator2.setPower(-0.9);
        //     elavator1.setPower(-0.9);
        // }
        // elavator2.setPower(0.0);
        // elavator1.setPower(0.0);

        while(opModeIsActive()){
            telemetry.addData("tar", straight.getMotor().getTargetPosition());
            telemetry.addData("cur", straight.getMotor().getCurrentPosition());
            telemetry.addData("zero", straight.getMotor().getCurrentPosition() - straight.getMotor().getTargetPosition());
            telemetry.update();
        }
        // while(opModeIsActive()){
        //     pos=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //     frontleftDrive.setPower(0.2);
        //     backleftDrive.setPower(0.2);
        //     frontrightDrive.setPower(-0.2);
        //     backrightDrive.setPower(-0.2);
        //     if ((pos < 5 && pos > -5) && forward==false){
        //         frontleftDrive.setPower(0);
        //         backleftDrive.setPower(0);
        //         frontrightDrive.setPower(0);
        //         backrightDrive.setPower(0);
        //         wait(5, runtime);
        //         forward=true;
        //         where=true;
        //     }
        //     if ((pos < -175 &&  pos > 175) && forward==true){
        //         frontleftDrive.setPower(0);
        //         backleftDrive.setPower(0);
        //         frontrightDrive.setPower(0);
        //         backrightDrive.setPower(0);
        //         wait(5, runtime);
        //         forward=false;
        //         where=true;
        //     }
        //     telemetry.addData("x", pos);
        //     telemetry.addData("forward", forward);
        //     telemetry.addData("cur",  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //     telemetry.addData("where", where);
        //     telemetry.update();
        // }
    }

    public void moveWithCorrection(double power, int target){
        double rl = imu.getRotationLeftPower(target);
        backLeftDrive.setPower(power + rl); //backR
        backRightDrive.setPower(power - rl); //frontL
        frontLeftDrive.setPower(power + rl);  //frontR
        frontRightDrive.setPower(power - rl);
    }

    private void waitMe(double sec){
        runtime.reset();
        while (runtime.seconds() < sec) {
        }
    }
}
