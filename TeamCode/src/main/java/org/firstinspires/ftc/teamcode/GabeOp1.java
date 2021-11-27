package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Gabe Op2")
public class GabeOp1 extends LinearOpMode {

    DcMotor lf, lr, rf, rr;
    DcMotor spinner, lift, extension;
    CRServo intake;

    public enum ArmStates{
        GROUND_COLLAPSED(0, 0), GROUND_EXPANDED(0, 1000),
        LOW_GOAL_COLLAPSED(100, 0), LOW_GOAL_EXTENDED(50, 1000),
        MID_GOAL_COLLAPSED(200, 0), MID_GOAL_EXTENDED(100, 1000),
        HIGH_GOAL_COLLAPSED(300, 0), HIGH_GOAL_EXTENDED(150, 1000);

        public int liftPosition, extensionPosition;

        ArmStates(int liftPos, int extensionPos){
            this.liftPosition = liftPos;
            this.extensionPosition = extensionPos;
        }
    }

    public void initMotors(){
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        spinner = hardwareMap.get(DcMotor.class, "spinner");
        lift = hardwareMap.get(DcMotor.class, "lift");
        extension = hardwareMap.get(DcMotor.class, "extension");

        intake = hardwareMap.get(CRServo.class, "topIntake");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void updateDrive(float fwd, float right, float rot){
        float deadzone = 0.1f;

        fwd = (Math.abs(fwd) > deadzone)? fwd : 0;
        right = (Math.abs(right) > deadzone)? right : 0;
        rot = (Math.abs(rot) > deadzone)? rot : 0;

        // calculate and set the power to each motor
        lf.setPower( fwd + right + rot );
        rf.setPower( fwd - right - rot);
        lr.setPower( fwd - right + rot );
        rr.setPower( fwd + right - rot );
    }
    public void updateArm(ArmStates armStates){
        float power = 1.0f;

        lift.setTargetPosition(armStates.liftPosition);
        extension.setTargetPosition(armStates.extensionPosition);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(power);
        extension.setPower(power);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors();



        waitForStart();

        while(opModeIsActive()){
            // Drive control
            updateDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Arm control
            if(gamepad1.a)
                updateArm(ArmStates.GROUND_COLLAPSED);
            else if(gamepad1.b)
                updateArm(ArmStates.LOW_GOAL_COLLAPSED);
            else if(gamepad1.x)
                updateArm(ArmStates.MID_GOAL_COLLAPSED);
            else if(gamepad1.y)
                updateArm(ArmStates.HIGH_GOAL_COLLAPSED);
            else if(gamepad1.dpad_down)
                updateArm(ArmStates.GROUND_EXPANDED);
            else if(gamepad1.dpad_right)
                updateArm(ArmStates.LOW_GOAL_EXTENDED);
            else if(gamepad1.dpad_left)
                updateArm(ArmStates.MID_GOAL_EXTENDED);
            else if(gamepad1.dpad_up)
                updateArm(ArmStates.HIGH_GOAL_EXTENDED);

            // Intake control
            if(gamepad1.right_bumper)
                intake.setPower(1.0f);
            else if(gamepad1.left_bumper)
                intake.setPower(-1.0f);
            else
                intake.setPower(0f);

            // Spinner control
            if(Math.abs(gamepad1.left_trigger) >= 0.25)
                spinner.setPower(1.0f);
            else if(Math.abs(gamepad1.right_trigger) >= 0.25)
                spinner.setPower(-1.0f);
            else
                spinner.setPower(0f);

        }
    }
}
