package org.firstinspires.ftc.teamcode.TeleOps.Drivebases.Swerve.CoaxialSwerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled

@TeleOp
public class CoaxialTeleOp extends LinearOpMode {

    private CRServo[] crServos;
    private DcMotor[] dcMotors;

    @Override
    public void runOpMode() {
        crServos = new CRServo[] {
                hardwareMap.crservo.get("mod0"),
//                hardwareMap.crservo.get("mod1"),
//                hardwareMap.crservo.get("mod2"),
//                hardwareMap.crservo.get("mod3"),
        };

        dcMotors = new DcMotor[] {
                hardwareMap.dcMotor.get("mod0m"),
//                hardwareMap.dcMotor.get("mod1m"),
//                hardwareMap.dcMotor.get("mod2m"),
//                hardwareMap.dcMotor.get("mod3m"),
        };

        telemetry.addLine("Initialization Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double joystickX = gamepad1.left_stick_x;
            double joystickY = gamepad1.left_stick_y;

            double angle = Math.atan2(joystickY, joystickX);

            mapAngleToCRServoPowers(angle);

            // Control the motor using the right joystick's Y position
            double motorPower = gamepad1.right_stick_y;
            for (int i = 0; i < dcMotors.length; i ++) {
                dcMotors[i].setPower(motorPower);
            }

            telemetry.addLine("Servo Positions:");
            for (int i = 0; i < crServos.length; i++) {
                telemetry.addData("Servo " + i, crServos[i].getPower());
            }
            telemetry.update();
        }
    }

    private void mapAngleToCRServoPowers(double angle) {
        // Map the angle to the CRServo power levels in the range [-1.0, 1.0]
        double minCRServoPower = -1.0; // Minimum CRServo power level
        double maxCRServoPower = 1.0;  // Maximum CRServo power level

        // Map the angle to the CRServo power levels for each CRServo
        for (int i = 0; i < crServos.length; i++) {
            double servoPower = angle / Math.PI; // Scale to [-1.0, 1.0]
            crServos[i].setPower(servoPower);
        }
    }
}

//    private void mapAngleToCRServoPowers(double angle) {
//        // Map the angle to the CRServo power levels in the range [-1.0, 1.0]
//        double minAngle = -Math.PI; // Minimum angle in radians
//        double maxAngle = Math.PI;  // Maximum angle in radians
//
//        double minCRServoPower = -1.0; // Minimum CRServo power level
//        double maxCRServoPower = 1.0;  // Maximum CRServo power level
//
//        // Map the angle to the CRServo power levels for each CRServo
//        for (int i = 0; i < crServos.length; i++) {
//            double servoPower = Range.scale(angle, minAngle, maxAngle, minCRServoPower, maxCRServoPower);
//            crServos[i].setPower(servoPower);
//        }
//    }