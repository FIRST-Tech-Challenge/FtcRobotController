package org.firstinspires.ftc.teamcode.zeroCode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.zeroCode.finalClassess.Constants;
import org.firstinspires.ftc.teamcode.zeroCode.finalClassess.FieldData;
import org.firstinspires.ftc.teamcode.zeroCode.utils.Intakes;
import org.firstinspires.ftc.teamcode.zeroCode.utils.Ramp;
import org.firstinspires.ftc.teamcode.zeroCode.utils.Turret;
import org.firstinspires.ftc.teamcode.zeroCode.utils.drive.DriveFC;
import org.firstinspires.ftc.teamcode.zeroCode.utils.drive.DriveRC;

import java.util.Locale;

@TeleOp(name="DrivebaseTesting", group="Testing")
public class DrivebaseTestingTeleOp extends OpMode {
    double oldTime = 0;
    DriveRC drive;
    Intakes intake;
    Ramp ramp;
    Turret turret;

    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        Constants.hardware.resetHardwareMaps(telemetry);
        //Constants.hardware.initializeIntakes = true;
        Constants.hardware.initializeDrive = true;
        Constants.hardware.initializePinpoint = true;
        //Constants.hardware.initializeRamp = true;
        //Constants.hardware.initializeIMU = true;
        //Constants.hardware.initializeTurret = true;
        Constants.hardware.initializeHardwareMaps(telemetry, hardwareMap);

        drive = new DriveRC();
        intake = new Intakes();
        ramp = new Ramp();
        //turret = new Turret();
        odo = Constants.pinpoint.pinpointDriver;

        telemetry.update();
    }

    @Override
    public void start() {
        super.start();

        resetRuntime();
    }

    @Override
    public void loop() {
        //Poll data
        odo.update();

        //Drive command:
        double[] motorPowers = drive.calculateDriveMecanumRC(
                //odo.getPosition().getHeading(AngleUnit.DEGREES),
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                -gamepad1.left_stick_y);

        drive.powerMotors(
                motorPowers[0],
                motorPowers[1],
                motorPowers[2],
                motorPowers[3]
        );

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
        telemetry.addData("Status", odo.getDeviceStatus());

        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

        /*
        if (gamepad1.x) {
            intake.powerPollenIntake(1);
        } else {
            intake.powerPollenIntake(0);
        }

        if (gamepad1.y) {
            intake.powerNectarIntake(1);
        } else {
            intake.powerNectarIntake(0);
        }


        if (gamepad1.a) {
            ramp.turnToNectar();
        } else if (gamepad1.b) {
            ramp.turnToNectar();
        }

        turret.powerTurret(
                gamepad1.left_trigger > gamepad1.right_trigger ? 
                        gamepad1.left_trigger : -gamepad1.right_trigger
        );

        telemetry.update();
        */
    }


}
