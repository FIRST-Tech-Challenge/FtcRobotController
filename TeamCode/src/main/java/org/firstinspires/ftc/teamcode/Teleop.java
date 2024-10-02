/**
 * Code is based on https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 * Mechanum drive TeleOp
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {

        private static final double H = 12.0;
    @Override
    public void runOpMode() throws InterruptedException {
        Bot bot = new Bot(this);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double pivot = -gamepad1.right_stick_x;

            double rightLiftTrigger = gamepad1.right_trigger; //rightTrigger is raising the lift
            double leftLiftTrigger = gamepad1.left_trigger; //leftTrigger is lowering the lift


            //when x button is pressed, rotates one way
            if (gamepad1.x) {
                bot.setIntakePosition(1.0);
            }
            //when y button is pressed, rotates the opposite way
            else if (gamepad1.y) {
                bot.setIntakePosition(-1.0);
            }
            //when no button is pressed, nothing rotates
            else {
                bot.setIntakePosition(0.0);
            }

            double frontLeftPower = y + x + pivot;
            double backLeftPower =  y - x + pivot;
            double frontRightPower = y - x - pivot;
            double backRightPower = y + x - pivot;
            if(rightLiftTrigger > 0.1){
                bot.setLift(1.0);//this makes it go down
            }
            else if(leftLiftTrigger > 0.1){
                bot.setLift(-1.0);//this makes it go up
            }
            else
            {
                bot.setLift(0.0);
            }
            //normalize power value
            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (max > 1.0){
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }
            bot.setDriveTrain(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            //extend arm controls
            if(gamepad1.right_bumper){
                bot.setExtendPower(1.0);
            } else if (gamepad1.left_bumper){
                bot.setExtendPower(-1.0);
            } else {
                bot.setExtendPower(0.0);
            }
            //auto arm positioning
//            double L = bot.getArmPosition();
//
//            if(L >= H){
//                double theta = Math.asin(H/L);
//                int maxEncoderTicks = 720; //for 180 degrees
//                double proportionOfFullRange = theta / Math.PI;
//                int targetPosition = (int)(proportionOfFullRange * maxEncoderTicks);
//
//                bot.autoPivotArm(targetPosition, 1.0);
//            } else {
//                bot.setPivotPower(0.0);
//            }
            if(gamepad1.a){
                bot.setPivotPower(0.75);
            } else if(gamepad1.b){
                bot.setPivotPower(-0.75);
            } else {
                bot.setPivotPower(0.0);
            }

            telemetry.addData("Current Length: ", L);
            telemetry.addData("Angle (Rads): ", L >= H ? Math.asin(H/L):"N/A");
            telemetry.addData("Horizontal Extension: ", L >= H? Math.sqrt(L*L-H*H):"N/A");
            telemetry.update();
        }
    }
}