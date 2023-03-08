package org.firstinspires.ftc.teamcode;

public class snippets {

    /*

            PIDFCoefficients pidfNew = new PIDFCoefficients(3,0,0,0);
            PIDFCoefficients pidf = robot.arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("original:", "%.04f, %.04f, %.04f, %.04f", pidf.p, pidf.i, pidf.d, pidf.f);
     */

            /*
            double Kp, Ki, Kd;
            double error = 0, lastError = 0, integralSum = 0;
            double out = 0;
            int cur = 0, reference = 30;
            ElapsedTime timer = new ElapsedTime();

            Kp = 0.005; Ki = 0; Kd = 0;


            if (gamepad1.right_bumper){
                timer.reset();
                while (1 == 1){
                    if (gamepad1.triangle){
                        reference += 10;
                        sleep(500);
                    }
                    cur = arm.getCurrentPosition();
                    telemetry.addData("Encoder value", arm.getCurrentPosition());
                    telemetry.update();
                    lastError = error;
                    error = reference-cur;
                    integralSum += error * timer.seconds();

                    //out = (error * Kp) + (integralSum * Ki) + ( ((error - lastError) / timer.seconds())  * Kd);

                    arm.setPower((error * Kp));

                    timer.reset();
                }

            }
            */ // custom PID

            /*
            while (gamepad1.dpad_up || gamepad1.dpad_down){
                if (gamepad1.dpad_up) pidfNew.p = pidfNew.p + 0.1;
                else if (gamepad1.dpad_down) pidfNew.p = pidfNew.p - 0.1;
            }

            if (gamepad1.right_bumper){
                robot.arm.setTargetPosition((int)(TICKS_PER_DEGREE * 100));
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.5);
            }
            */ // rev PID with setVelocity() and custom PIDF coefficients

}
