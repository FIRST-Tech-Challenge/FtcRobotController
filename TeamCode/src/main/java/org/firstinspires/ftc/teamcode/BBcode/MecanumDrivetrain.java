package org.firstinspires.ftc.teamcode.BBcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrivetrain {
    OpMode _opMode;
    TelemetryHelper _telemetryHelper;
    DcMotorEx _leftFront;
    DcMotorEx _leftBack;
    DcMotorEx _rightFront;
    DcMotorEx _rightBack;
    // Constructor
    public MecanumDrivetrain(OpMode opMode) {
        _opMode = opMode;
        _telemetryHelper = new TelemetryHelper(opMode);
        // Initialize the motors
        _leftFront = _opMode.hardwareMap.tryGet(DcMotorEx.class, "leftFront");
        _leftBack = _opMode.hardwareMap.tryGet(DcMotorEx.class, "leftBack");
        _rightFront = _opMode.hardwareMap.tryGet(DcMotorEx.class, "rightFront");
        _rightBack = _opMode.hardwareMap.tryGet(DcMotorEx.class, "rightBack");

        //For right now, just add a telemetry message but the code will still fail when it's accessed in code so gracefully handle the null case
        //This could be to exit the OpMode or to continue with the OpMode but not use the device. The latter requires checking for null in the code
        if (_leftFront == null || _leftBack == null || _rightFront == null || _rightBack == null)
        {
            _opMode.telemetry.addLine("One or more motors not found!");
        } else {
            // Reverse the right side motors
            _rightFront.setDirection(DcMotor.Direction.REVERSE);
            _rightBack.setDirection(DcMotor.Direction.REVERSE);
        }

        _telemetryHelper.initMotorTelemetry( _leftFront, "LF");
        _telemetryHelper.initMotorTelemetry( _leftBack, "LB");
        _telemetryHelper.initMotorTelemetry( _rightFront, "RF");
        _telemetryHelper.initMotorTelemetry( _rightBack, "RB");
    }

    public void Drive() {
        double drive;
        double turn;
        double strafe;
        double fLeftPow, fRightPow, bLeftPow, bRightPow;
        double speedMultiplier = 0.75;
        double turnEasingExponet = 3, turnEasingYIntercept = 0.05;

        Gamepad gamepad1 = _opMode.gamepad1;
        //drive inputs
        drive = gamepad1.left_stick_y;
        turn = Math.pow((gamepad1.right_stick_x * -1), turnEasingExponet) + (Math.signum(gamepad1.right_stick_x * -1) * turnEasingYIntercept);
        strafe = gamepad1.left_stick_x * -1;
        if (gamepad1.left_trigger > 0) {speedMultiplier = 0.25;}

        boolean isDriveDisabled = false; //Debugging flag to disable drive code
        if (!isDriveDisabled) {
            //drive calculations

            fLeftPow = Range.clip((drive + turn + strafe) * speedMultiplier, -1, 1);
            bLeftPow = Range.clip((drive + turn - strafe) * speedMultiplier, -1, 1);
            fRightPow = Range.clip((drive - turn - strafe) * speedMultiplier, -1, 1);
            bRightPow = Range.clip((drive - turn + strafe) * speedMultiplier, -1, 1);

            if (_leftFront != null) {_leftFront.setPower(fLeftPow);}
            if (_leftBack != null) {_leftBack.setPower(bLeftPow);}
            if (_rightFront != null) {_rightFront.setPower(fRightPow);}
            if (_rightBack != null) {_rightBack.setPower(bRightPow);}
        }
    }
}
