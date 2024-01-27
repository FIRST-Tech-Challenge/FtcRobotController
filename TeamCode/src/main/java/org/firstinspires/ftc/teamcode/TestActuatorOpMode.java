package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.ActuatorCommand;
import org.firstinspires.ftc.teamcode.commands.ActuatorCommandReverse;
import org.firstinspires.ftc.teamcode.commands.ActuatorCommandStop;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCom;
import org.firstinspires.ftc.teamcode.subbys.ActuatorSubby;
import org.firstinspires.ftc.teamcode.subbys.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subbys.ShooterSubby;

@TeleOp(name = "picapicapicapica")
public class TestActuatorOpMode extends CommandOpMode {
//    private Motor hang, arm;
    private Motor arm;
    private Motor hang;
//    private SensorRevTOFDistance sensor;

//    private ActuatorSubby actuator;
//    private ActuatorCommand actuator_com1;
//    private ActuatorCommandReverse actuator_com2;

//    private ActuatorCommandStop stop;
    private Motor fL, bL, fR, bR;
    private RevIMU imu;
    private SimpleServo left_claw;
    private SimpleServo right_claw;
    private SimpleServo shooter;
//    private SimpleServo turner;

    private DriveSubsystem driveS;
    private DriveCommand driveC;

    private GamepadEx gpad;
    @Override
    public void initialize() {
        hang = new Motor(hardwareMap, "hang");
//        sensor = new SensorRevTOFDistance(hardwareMap, "commonSense");
        arm = new Motor(hardwareMap, "arm");
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        imu = new RevIMU(hardwareMap);
        imu.init();
//
        shooter = new SimpleServo(hardwareMap, "s", -180, 180);
        left_claw = new SimpleServo(hardwareMap, "lc", -180, 180);
        right_claw = new SimpleServo(hardwareMap, "rc", -180, 180);
//        turner = new SimpleServo(hardwareMap, "t", -360, 0);

//
        fL.motor.setDirection(DcMotor.Direction.FORWARD);
        fR.motor.setDirection(DcMotor.Direction.FORWARD);
        bL.motor.setDirection(DcMotor.Direction.FORWARD);
        bR.motor.setDirection(DcMotor.Direction.FORWARD);

        arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
////        right_claw.setInverted(true);
//
        fL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fL.motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fR.motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bL.motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bR.motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        gpad = new GamepadEx(gamepad1);

//        actuator = new ActuatorSubby(hang, sensor);
//        actuator_com1 = new ActuatorCommand(actuator);
//        actuator_com2 = new ActuatorCommandReverse(actuator);
//        stop = new ActuatorCommandStop(actuator);

//        gpad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(actuator_com1).whenReleased(stop);
//        gpad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(actuator_com2).whenReleased(stop);

        gpad.getGamepadButton(GamepadKeys.Button.X).whenHeld(
                new InstantCommand(() -> arm.set(0.45))).whenReleased(new InstantCommand(() -> arm.set(0)));

        gpad.getGamepadButton(GamepadKeys.Button.B).whenHeld(
                new InstantCommand(() -> arm.set(-0.45))).whenReleased(new InstantCommand(() -> arm.set(0)));

        gpad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(
                new InstantCommand(() -> hang.set(1))).whenReleased(new InstantCommand(() -> hang.set(0)));

        gpad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(
                new InstantCommand(() -> hang.set(-1))).whenReleased(new InstantCommand(() -> hang.set(0)));

        gpad.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                new InstantCommand(() -> arm.set(-0.65)),
                new InstantCommand(() -> arm.set(0))
        );
        gpad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> shooter.setPosition(1))
                        .andThen(new WaitCommand(200))
                        .andThen( new InstantCommand(() -> shooter.setPosition(0)))
                        .andThen( new InstantCommand(() -> shooter.setPosition(-1)))
        );
//                new InstantCommand(() -> shooter.setPosition(-1))
//                        .andThen(new WaitCommand(400))
//                        .andThen( new InstantCommand(() -> shooter.setPosition(0)))
//        );
//
        gpad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(
                new RunCommand(() -> left_claw.turnToAngle(-10)).andThen(), //95
                new RunCommand(() -> left_claw.turnToAngle(-85)) //5
        );
//
        gpad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(
                new InstantCommand(() -> right_claw.turnToAngle(-160)),//90
                new InstantCommand(() -> right_claw.turnToAngle(-70)) //0
        );
//
//        gpad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(
//                new RunCommand(() -> left_claw.turnToAngle(15)).andThen(new RunCommand(() -> right_claw.turnToAngle(-100))),
//                new RunCommand(() -> left_claw.turnToAngle(105)).andThen(new RunCommand(() -> right_claw.turnToAngle(-10)))
//        );
//        gpad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(
//                new InstantCommand(() -> left_claw.turnToAngle(-115)).andThen(new InstantCommand(() -> right_claw.turnToAngle(30))),
//                new InstantCommand(() -> left_claw.turnToAngle(100)).andThen(new InstantCommand(() -> right_claw.turnToAngle(-70)))
//        );

//        gpad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new InstantCommand(() -> turner.rotateByAngle(10))
//        );
//        gpad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new InstantCommand(() -> turner.rotateByAngle(-10))
//        );

//        gpad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(() -> left_claw.rotateByAngle(10))
//        );
//        gpad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                new InstantCommand(() -> left_claw.rotateByAngle(-10))
//        );



        driveS = new DriveSubsystem(fL, fR, bL, bR, imu, 1.0);
        driveC = new DriveCommand(driveS, gpad::getLeftX, gpad::getLeftY, gpad::getRightX, 0.65);

//        gpad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
//                () -> new DriveCommand(driveS, gpad::getLeftX, gpad::getLeftY, gpad::getRightX, 0.5),
//                () -> new DriveCommand(driveS, gpad::getLeftX, gpad::getLeftY, gpad::getRightX, 1.0)
//        );

        register(driveS);
        driveS.setDefaultCommand(driveC);

        schedule(new RunCommand(() ->{
//            telemetry.addData("inches: ", sensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("right: ", right_claw.getAngle());
//            telemetry.addData("left: ", left_claw.getAngle());
            telemetry.update();
        }));

    }

}
