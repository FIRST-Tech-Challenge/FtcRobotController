

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Field Centric")
public class FieldCentricTeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor outtake;
    private DcMotor intake;
    private CRServo left;
    private CRServo right;
    private DcMotor transfer;

    //pinpoint
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode(){


        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class ,"rb");
        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");
        odo =  hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(1, -6, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);



        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("it works");

        waitForStart();
        if(isStopRequested()) return;

        boolean prevA = false;
        boolean stateA = false;

        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double ry = -gamepad1.right_stick_y;
            double rx = gamepad1.right_stick_x;
            boolean padA = gamepad1.a;
            if(padA != prevA){
                if (padA) {
                    stateA = true;
                } else {
                    stateA = false;
                }
            }
            prevA = padA;

            odo.update();
            double heading = odo.getHeading(AngleUnit.RADIANS);
            /* Used later for properly rotating Cartesian coordinates (normal coordinate plane)
            mathematically using a certain heading, which tells you the angle difference between
            the current location versus the desired location via rotation by assuming the current
            location is at 0 radians everytime. */
            double theta = odo.getHeading(AngleUnit.DEGREES)-180; //Same as heading but in degrees, used for the following code
            if (theta < 0) {
                theta = 360 + theta;} //makes theta range from the built-in -180 - 180 range to 0 - 360 range
            double pointer = Math.toDegrees(Math.atan2 (ry,rx)); //Converts right joystick motion into degrees (vector),
            // atan2 is the same as taking the arctan for two arguments (inputs).
            if (pointer < 0) {
                pointer = 360 + pointer;} //0-360 range like theta
            double difference = pointer - theta; //Difference checks the closest way to get to your location
            difference = difference - 360.0 * Math.round(difference / 360.0); //Makes difference negative so that it can go -90 turn instead of 270 for instance.
            double magnitude = Math.hypot(rx,ry); /* Checks how much has the joystick
            been offset from the positon where you are not pushing the joystick*/
            double r = 0; // Makes the rate of turning based on where the desired rotation location is
            if (stateA) { //Advanced mode
                if (magnitude > 0.2) { //Checks if we are actually actively trying to turn it
                    r = Math.round(difference/180);} //Signum basically takes the sign only, so it can directly go to the directed result
            } else {
                r=rx;} //Basic mode
            //The above code basically turns the robot to the direction the right controller is pointing, so it stays relative to the driver

            double rotx = x*Math.cos(-heading)-y*Math.sin(-heading);
            double roty = x*Math.sin(-heading)+y*Math.cos(-heading);
            //Math behind rotation and changing the x and y coordinates accordingly
            double lfPower = roty + rotx + r;
            double lbPower = roty - rotx + r;
            double rfPower = roty - rotx - r;
            double rbPower = roty + rotx - r;
            //Mecanum power formulas, it just works, oui oui baguette
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            if(gamepad1.left_trigger_pressed) {
                outtake.setPower(0.9);
            } else if(gamepad1.right_trigger_pressed) {
                outtake.setPower(0.9);
                transfer.setPower(1);
            } else {
                outtake.setPower(0);
                transfer.setPower(0);

            }
            /*
            controls:
            A: Intake transfer max speed
            B: Transfer slightly slower
            X: Reverse
            Left Trigger: Outtake only
            Right Trigger: Outtake and transfer
             */

            if(gamepad1.a) {
                intake.setPower(1);
                left.setPower(1);
                right.setPower(-1);
                transfer.setPower(1);
            } else if(gamepad1.b) {
                intake.setPower(1);
                left.setPower(1);
                right.setPower(-1);
                transfer.setPower(0.8);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
                left.setPower(0);
                right.setPower(0);
            }

            if(gamepad1.x) {
                transfer.setPower(-1);
                intake.setPower(-1);
                left.setPower(-1);
                right.setPower(1);
            }


            telemetry.addLine("its running");
            telemetry.addData("stateA: ", stateA);
            telemetry.update();

            telemetry.update();
        }




    }
}