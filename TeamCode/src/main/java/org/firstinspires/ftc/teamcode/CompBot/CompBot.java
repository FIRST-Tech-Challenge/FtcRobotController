package org.firstinspires.ftc.teamcode.CompBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

@TeleOp(name = "CompBot Swerve", group = "CompBot")
public class CompBot extends LinearOpMode {

    SwerveConfig swerve = new SwerveConfig(this);
    MekanismConfig mek = new MekanismConfig(this);
    Utils utils = new Utils(this);
    GraphicTelemetry graph = new GraphicTelemetry(this);


    GoBildaPinpointDriver odo;

    DcMotor FLMotor, BLMotor, BRMotor, FRMotor, pivot, slide;

    Servo FLServo, BLServo, BRServo, FRServo, claw;


    // In case builders are bad, is offset center for servo
    double FLServoOffSet = .00;     //0.00
    double FRServoOffSet = .00;     //0.00
    double BLServoOffSet = .00;     //0.01
    double BRServoOffSet = .00;     //.007


    /**
     * Controls for Gamepad 1:
     * Right trigger: Forwards
     * Left trigger: Reverse
     * Right stick X: Rotate
     * Left stick X Strafe
     * <p>
     * Controls for Gamepad 2:
     * Left stick y: In and out of arm
     * Right stick y: Up and down of arm
     * Left trigger: Claw intake
     * Right trigger: Claw out
     * Presets for:
     * Attaching clip to sample
     * Attaching specimen(clip + sample) to top rung
     * Presets for bucket 1 and 2
     */
    public void runOpMode() throws InterruptedException {

        swerve.initSwerve(); // Inits all the stuff related to swerve drive
        mek.initMekanism(); // Inits the mechanism stuff


        waitForStart();
        while (opModeIsActive()) {

            // Gamepad 1
            double speedGMP1 = gamepad1.left_trigger - gamepad1.right_trigger; // Makes it so that the triggers cancel each other out if both are pulled at the same time
            double angleGMP1 = -gamepad1.right_stick_x;

            if (speedGMP1 != 0) swerve.moveStraight(gamepad1.left_stick_x, speedGMP1);
            else if (angleGMP1 != 0) swerve.rotate(angleGMP1);
            else {
                FLMotor.setPower(0);
                BLMotor.setPower(0);
                BRMotor.setPower(0);
                FRMotor.setPower(0);
            }


            // Gamepad 2
            double armLength = -gamepad2.right_stick_y;
            double armAngle = -gamepad2.left_stick_y;


            // Moves arm
            mek.basicMoveArm(armLength, armAngle);

        }
    }


    // To test arm lift and extend
    // TODO: Get rid of this
    public void addTelem(int x, int y, double a, double b) {
        telemetry.addData("Arm angle: ", x);
        telemetry.addData("Arm length: ", y);
        telemetry.addData("Left stick y: ", a);
        telemetry.addData("Right stick y: ", b);
        telemetry.addData("Pivot target pos: ", pivot.getTargetPosition());
        telemetry.addData("Arm target pos: ", slide.getTargetPosition());
        telemetry.addData("Pivot power: ", pivot.getPower());
        telemetry.addData("Arm power: ", slide.getPower());
        telemetry.addData("Pivot position: ", pivot.getCurrentPosition());
        telemetry.addData("Arm position: ", slide.getCurrentPosition());
        telemetry.update();
    }

}
