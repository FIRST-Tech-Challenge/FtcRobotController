package org.firstinspires.ftc.teamcode.CompBotV3;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous
public class EncoderDriveVerification extends LinearOpMode {
    CompBotV3Attachments r = new CompBotV3Attachments();
    @Override
    public void runOpMode() throws InterruptedException {
        double dForward = 6, dStrafe = 3, sForward = 0.2, sStrafe = 0.2, distanceK = 384.5/(100*Math.PI)*25.4, corrCoeff = 0, dervCoeff = 0, intCoeff = 0;

        r.init(hardwareMap);

        // Set the target positions of each motor
        r.fl.setTargetPosition(r.fl.getCurrentPosition() + (int) -(distanceK*(dForward+dStrafe))); // distanceK is a conversion factor to convert linear distance to motor clicks;
        r.fr.setTargetPosition(r.fr.getCurrentPosition() + (int) (distanceK*(dForward-dStrafe))); // The distance each wheel needs to travel is just the sum of the
        r.bl.setTargetPosition(r.bl.getCurrentPosition() + (int) -(distanceK*(dForward-dStrafe))); // distances the wheel would need to travel to do the strafing and
        r.br.setTargetPosition(r.br.getCurrentPosition() + (int) (distanceK*(dForward+dStrafe))); // forward/back distances separately
        r.runToPositionMode(); // Set motors to RUN_TO_POSITION mode - they will automatically spin in the direction of the set position
        telemetry.addLine("mpos: ("+r.fl.getTargetPosition()+","+r.fr.getTargetPosition()+","+r.bl.getTargetPosition()+","+r.br.getTargetPosition()+")");
        telemetry.update();
        waitForStart();
        double initialHeading = r.imu.getHeading(); // Create variables for the gyro correction, and measure the initial angle the robot is facing
        double pastError = 0, intError = 0;
        ElapsedTime e = new ElapsedTime();
        while((r.fl.isBusy() || r.fr.isBusy() || r.bl.isBusy() || r.br.isBusy())&& !isStopRequested()) {
            double elapsedTime = e.milliseconds()/1000;
            e.reset(); // Reset the timer
            double error = r.imu.getHeading() - initialHeading; // Calculate the deviation from the initial angle of the robot using the gyro
            double dervError = (error-pastError)/elapsedTime; // Calculate the derivative = rate of change of the error
            intError += error*elapsedTime; // Calculate the integral = sum over time of error
            double totalError = corrCoeff*error + dervCoeff*dervError + intCoeff*intError; // Sum the errors and apply coefficients
            if(r.fl.isBusy()) { r.fl.setPower(-(sForward + sStrafe) - totalError); // DCMotor.isBusy is a boolean variable signifying whether the motor has finished moving to the position
            } else { r.fl.setPower(MathUtils.clamp(Math.signum(r.fl.getCurrentPosition()-r.fl.getTargetPosition())*-1*totalError, -1, 1));
            }if(r.fr.isBusy()) { r.fr.setPower(sForward - sStrafe + totalError); // This code looks complicated but it's simple
            } else { r.fr.setPower(MathUtils.clamp(Math.signum(r.fr.getCurrentPosition()-r.fr.getTargetPosition())*totalError,-1,1));
            }if(r.bl.isBusy()) { r.bl.setPower(-(sForward - sStrafe) - totalError); // If the motor is not finished, apply the given speed + a correction based on the angle error
            } else { r.bl.setPower(MathUtils.clamp(Math.signum(r.bl.getCurrentPosition()-r.bl.getTargetPosition())*-1*totalError,-1,1));
            } if(r.br.isBusy()) { r.br.setPower(sForward + sStrafe + totalError); // If the motor is finished, apply only the correction (flip flops signs because we're in RUN_TO_POSITION mode)
            } else { r.br.setPower(MathUtils.clamp(Math.signum(r.br.getCurrentPosition()-r.br.getTargetPosition())*totalError,-1,1));
            }
            pastError = error; // Move error into pastError for next loop
            telemetry.addData("initial heading", initialHeading);
            telemetry.addData("current heading", r.imu.getHeading());
            telemetry.addData("prop error",error);
            telemetry.addData("dervError", dervError);
            telemetry.addData("total error", totalError);
            telemetry.addLine("mpos: ("+ Arrays.toString(new int[]{r.fl.getCurrentPosition(),r.fr.getCurrentPosition(),r.bl.getCurrentPosition(),r.br.getCurrentPosition()}));
            telemetry.addLine(Arrays.toString(new boolean[]{r.fl.isBusy(), r.fr.isBusy(), r.bl.isBusy(), r.br.isBusy()}));
            telemetry.addLine("MP: ("+r.fl.getPower()+","+r.fr.getPower()+","+r.bl.getPower()+","+r.br.getPower()+")");
            telemetry.update();
        }
        r.stop();
        r.useEncoders(); // Switch back to normal RUN_USING_ENCODERS velocity control mode

    }
}
