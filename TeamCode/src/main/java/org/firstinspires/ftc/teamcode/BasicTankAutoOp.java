package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
Omni-wheel encoder based test on tank chassis using RoadRunner
 */

@Autonomous (name = "Basic Tank Auto")
public class BasicTankAutoOp extends LinearOpMode{
    private DcMotor motorRF;
    private DcMotor motorLB;
    private DcMotor motorLF;
    private DcMotor motorRB;

    int drivePercentage = 100;

    enum TankSide {
        LEFT,
        RIGHT
    }

    @Override
    public void runOpMode() {
        boolean runTimeBased = true;
        boolean runMotorEncoderRotationCount = false;

        motorRF = hardwareMap.get(DcMotor.class, "motorRF");
        motorLB = hardwareMap.get(DcMotor.class, "motorLB");
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (runTimeBased) {
            // Time based Motion
            setDrivePower(TankSide.LEFT, -0.25f);
            setDrivePower(TankSide.RIGHT, -0.25f);

            telemetry.addData("Power", "LF: %1$s, RB: %2$s" ,motorLF.getPower(), motorRB.getPower());
            telemetry.addData("Status", "Running - Time Based Motion");
            telemetry.update();

            sleep(3000);

            haltRobot();

            telemetry.addData("Power", "LF: %1$s, RB: %2$s" ,motorLF.getPower(), motorRB.getPower());
            telemetry.addData("Status", "Paused");
            telemetry.update();

            sleep(3000);
        }

        if (runMotorEncoderRotationCount) {
            // Motor Encoder Rotation Count
            //
            // Motor RB/LB are both using direct motor encoders
            motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int posRB = motorRB.getCurrentPosition();
            int targetPosRB = posRB + 4000;
            motorRB.setTargetPosition(targetPosRB);

            motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Status", "Running - Motor Encoder Rotation Count");
            telemetry.update();

            //setDrivePower(TankSide.LEFT, -0.25f);
            setDrivePower(TankSide.RIGHT, -0.25f);

            while (opModeIsActive() &&
                    motorRB.isBusy()) {
                posRB = motorRB.getCurrentPosition();

                telemetry.addData("Position", "motorRB: %1$d, Target: %2$d", posRB, targetPosRB);
                telemetry.update();
            }
        }
    }

    /**
     * Adjust the value of power by the value of percent
     * This helps customize the speed of motors when using thumbstick controls
     */
    private double powerAdjust(double power, double percent) {
        double result;

        result = (power * percent) / 100;
        return result;
    }

    /**
     * Set both motors on a tank side to power
     * @param side tank side
     * @param power motor power
     */
    private void setDrivePower(TankSide side, float power) {
        double adjPower = powerAdjust(power, drivePercentage);
        if (TankSide.LEFT == side) {
            motorLF.setPower(adjPower);
            motorLB.setPower(adjPower);
        } else {
            motorRF.setPower(adjPower);
            motorRB.setPower(adjPower);
        }
    }

    /**
     * Halt robot by setting motor power to zero.
     */
    private void haltRobot() {
        setDrivePower(TankSide.LEFT, 0);
        setDrivePower(TankSide.RIGHT, 0);
    }
}
