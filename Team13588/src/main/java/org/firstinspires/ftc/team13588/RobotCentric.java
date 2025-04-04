package org.firstinspires.ftc.team13588;

/*
*this OpMode Illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
* This approach is very efficient because the same hardware class can be used by all of your teleOp and autonomous OpModes
* without requiring many copy & paste operations. Once you have defined and tested the hardware class with one OpMode,
* It is instantly available to other OpMOdes.
*
* The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place
* (the Hardware class). So, to be effective you should put as much or your hardware setup and access code as possible in
* the hardware class.
*
* The Hardware class is created in a separate file, and then an "instance" of this class is created in each OpMode.
* In order for the class to do typical OpMode things (like send telemetry data) It must be passed a reference to the
* OpMode object when it's created, so it can access all core OpMode functions.
*
*
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name= "Concept: Robot Hardware Class", group= "Robot")

public class RobotCentric extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    //Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double axial       = 0;
        double lateral     = 0;
        double yaw         = 0;

        //initialize all the hardware using the hardware class. See this? Very simple. Very clean. Very demure.
        robot.init();

        //robot.intake.setPower(INTAKE_OFF);

        //Send telemetry message to simplify robot waiting;
        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        //robot.armPosition = RobotHardware.ARM_WINCH_ROBOT;
        //robot.liftPosition = RobotHardware.RETRACT;
        //robot.wrist.setPosition(-robot.WRIST_FOLDED_IN);


        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            * Run wheels in POV mode (not: The joystick goes negative when pushed forward, so we'll negate it)
            * In this mode the left stick moves the robot fwd and back, the Right stick turns left and right.
            * This way it's also easy to just drive straight, or just turn.
             */
            axial= gamepad1.left_stick_y * 0.74;
            lateral= gamepad1.left_stick_x * 0.75;
            yaw= -gamepad1.right_stick_x * 0.75;

            //combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(axial, lateral, -yaw);

            /* TECH TIP: If Else statements:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intake will turn on. If we just had
            three if statements, then it will set the intake servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */

            if (gamepad1.a) {
                robot.intake.setPower(RobotHardware.INTAKE_COLLECT);
            }
            else if (gamepad1.x) {
                robot.intake.setPower(RobotHardware.INTAKE_OFF);
            }
            else if (gamepad1.b) {
                robot.intake.setPower(RobotHardware.INTAKE_DEPOSIT);
            }





            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

           /* if(gamepad1.right_bumper){
                //This is the intaking/collecting arm position
                robot.armPosition = robot.ARM_COLLECT;
                robot.wrist.setPosition(robot.WRIST_FOLDED_OUT);
                robot.intake.setPower(robot.INTAKE_COLLECT);
            }*/

            if (gamepad1.left_bumper){
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
                robot.armPosition = RobotHardware.ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.y){
                /* This is the correct height to score the sample in the LOW BASKET */
                robot.armPosition = RobotHardware.ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                robot.armPosition = RobotHardware.ARM_COLLAPSED_INTO_ROBOT;
                robot.intake.setPower(RobotHardware.INTAKE_OFF);
                robot.wrist.setPosition(RobotHardware.WRIST_FOLDED_OUT);
            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                robot.armPosition = RobotHardware.ARM_SCORE_SPECIMEN;
                robot.wrist.setPosition(RobotHardware.WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                robot.armPosition = RobotHardware.ARM_ATTACH_HANGING_HOOK;
                robot.intake.setPower(RobotHardware.INTAKE_OFF);
                robot.wrist.setPosition(RobotHardware.WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                robot.armPosition = RobotHardware.ARM_COLLAPSED_INTO_ROBOT;
                robot.intake.setPower(RobotHardware.INTAKE_OFF);
                robot.wrist.setPosition(RobotHardware.WRIST_FOLDED_IN);
            }

            // gamepad2 controls;
            // y extends the arm
            if(gamepad2.y){
                //robot.armPosition = robot.ARM_WINCH_ROBOT;
                robot.liftPosition = RobotHardware.EXTEND;
            }
            // x retracts the arm
           else if (gamepad2.x) {
             //   robot.armPosition = robot.ARM_WINCH_ROBOT;
               robot.liftPosition = -RobotHardware.EXTEND;

            }
            else if(gamepad2.a){
                robot.armPosition = RobotHardware.ARM_WINCH_ROBOT;
                robot.liftPosition = -RobotHardware.RETRACT;
            }


            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            robot.armPositionFudgeFactor = RobotHardware.FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));


            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            robot.armDrive.setTargetPosition((int) (robot.armPosition + robot.armPositionFudgeFactor));

            ((DcMotorEx) robot.armDrive).setVelocity(2100);
            robot.armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */

            // doing the same for the liftDriveliftDrive motor
            robot.liftDrive.setTargetPosition((int) (robot.liftPosition + robot.armPositionFudgeFactor));
            ((DcMotorEx)  robot.liftDrive).setVelocity(500);
            robot.liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) robot.armDrive).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", robot.armDrive.getTargetPosition());
            telemetry.addData("arm Encoder: ", robot.armDrive.getCurrentPosition());
            telemetry.update();

            //Send telemetry messages to explain controls and show robot status
            telemetry.addData("Axial", "left_stick_y");   // Axial means driving up and down
            telemetry.addData("Lateral", "left_stick_x");     // lateral means driving side to side
            telemetry.addData("Yaw", "right_stick_x"); // Don't be afraid. Yaw means Turning
            telemetry.addData("-", "--------");

             telemetry.addData("Axial Power", "%2f", axial);
             telemetry.addData("Lateral Power ", "%2f", lateral);
             telemetry.addData("Yaw Power", "%2f", yaw);
             telemetry.update();

             // We'll pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
