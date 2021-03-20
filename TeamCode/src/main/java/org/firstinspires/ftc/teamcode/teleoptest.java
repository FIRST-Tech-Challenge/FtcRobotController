package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="teleoptest", group="UltimateGoal")
public class teleoptest extends OpMode {

    // declaring variables
    MecanumDriveTrain vroom;

    /* Declare OpMode members. */
    HardwareMapV2 robot = new HardwareMapV2(false); // use the class created to define a RoverRuckus's hardware

    @Override
    public void init() {
        telemetry.addData("What", "Do u want");
        telemetry.update();
        robot.init(hardwareMap);
//        robot.setEncoders(robot.motors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Set to REVERSE if using AndyMark motors
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);// Set to FORWARD if using AndyMark motors
//        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.setEncoders(robot.motors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vroom = new MecanumDriveTrain(robot, gamepad1,telemetry);

        telemetry.addData("Haddi", "Haddi");
        telemetry.update();
    }

    @Override
    public void loop() {
        vroom.loop();
    }
}
