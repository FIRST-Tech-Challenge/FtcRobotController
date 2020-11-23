package org.firstinspires.ftc.teamcode.Qualifier_1.Components;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;

        import java.io.File;
        import java.io.FileWriter;
        import java.io.IOException;

        import static java.lang.Math.PI;
        import static java.lang.Math.abs;
        import static java.lang.Math.atan2;
        import static java.lang.Math.cos;
        import static java.lang.Math.sin;
        import static java.lang.Math.sqrt;
public class OdometryChassis extends BasicChassis {
    double[] encoder = new double[4];
    double xpos = 0;
    double ypos = 0;
    private LinearOpMode op = null;
    private Odometry odom = null;
    public OdometryChassis(LinearOpMode opMode) {
        super(opMode);
        odom = new Odometry();
    }
    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public double[] track() {
        return odom.track();
    }
    public double getAngle() {
        return odom.getAngle();
    }
    public void turnInPlace(double target, double power) {
        double currentAngle = odom.getAngle();
        int direction = 1;
        double difference = currentAngle - target;
        double targetAngle = target;
        if (target < 0) {
            direction = -1;
        }
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.5)) {
            currentAngle = odom.getAngle();
            difference = targetAngle - currentAngle;
            if (difference * direction < 5) {
                power *= difference / 5;
                if (power < 0.2) {
                    power = 0.2 * power;
                }
            }
            motorRightBack.setPower(-power * direction);
            motorRightFront.setPower(-power * direction);
            motorLeftBack.setPower(power * direction);
            motorLeftFront.setPower(power * direction);
            op.telemetry.addData("current angle", currentAngle);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    public void moveForward(double distance, double power) {
        double[] currentPosition = odom.track();
        double[] target_position = {0, 0, 0};
        double correction = 0;
        double anglecorrection = 0;
        int direction = 1;
        target_position[0] = currentPosition[0] + sin(odom.getAngle() * PI / 180) * distance;
        target_position[1] = currentPosition[1] + cos(odom.getAngle() * PI / 180) * distance;
        target_position[2] = currentPosition[2];
        if (distance < 0) {
            direction = -1;
        }
        double difference = distance;
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.25)) {
            currentPosition = odom.track();
            difference = abs(target_position[1] - currentPosition[1]);
            anglecorrection = (currentPosition[2] - target_position[2]) * .005;
            if (difference * direction < 5) {
                power *= difference / 5;
                if (power < 0.2) {
                    power = 0.3 * power;
                }
            }
            motorRightBack.setPower(-power * direction + anglecorrection + anglecorrection);
            motorRightFront.setPower(-power * direction + anglecorrection - anglecorrection);
            motorLeftBack.setPower(-power * direction - anglecorrection - anglecorrection);
            motorLeftFront.setPower(-power * direction - anglecorrection + anglecorrection);
            op.telemetry.addData("current xpos", currentPosition[0] + "current ypos", currentPosition[1]);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public void moveBackward(double distance, double power) {
        double[] currentPosition = odom.track();
        double[] target_position = {0, 0, 0};
        double correction = 0;
        double anglecorrection = 0;
        int direction = 1;
        target_position[0] = currentPosition[0] + sin(odom.getAngle() * PI / 180) * -distance;
        target_position[1] = currentPosition[1] + cos(odom.getAngle() * PI / 180) * -distance;
        target_position[2] = currentPosition[2];
        if (distance < 0) {
            direction = -1;
        }
        double difference = distance;
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.25)) {
            currentPosition = odom.track();
            difference = abs(target_position[1] - currentPosition[1]);
            anglecorrection = (currentPosition[2] - target_position[2]) * .005;
            if (difference * direction < 5) {
                power *= difference / 5;
                if (power < 0.2) {
                    power = 0.3 * power;
                }
            }
            motorRightBack.setPower(-power * direction + anglecorrection + anglecorrection);
            motorRightFront.setPower(-power * direction + anglecorrection - anglecorrection);
            motorLeftBack.setPower(-power * direction - anglecorrection - anglecorrection);
            motorLeftFront.setPower(-power * direction - anglecorrection + anglecorrection);
            op.telemetry.addData("current xpos", currentPosition[0] + "current ypos", currentPosition[1]);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public void moveRight(double distance, double power) {//right is positive use distance to change direction
        double[] currentPosition = odom.track();
        double[] target_position = {0, 0, 0};
        double correction = 0;
        double anglecorrection = 0;
        int direction = 1;
        target_position[0] = currentPosition[0] + cos(odom.getAngle() * PI / 180) * distance;
        target_position[1] = currentPosition[1] + sin(odom.getAngle() * PI / 180) * distance;
        target_position[2] = currentPosition[2];
        if (distance < 0) {
            direction = -1;
        }
        double difference = distance;
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.25)) {
            currentPosition = odom.track();
            difference = abs(target_position[0] - currentPosition[0]);
            anglecorrection = (currentPosition[2] - target_position[2]) * 0.005;
            if (difference * direction < 5) {
                power *= difference / 5;
                if (power < 0.2) {
                    power = 0.3 * power;
                }
            }
            motorRightBack.setPower(-power * direction - anglecorrection - anglecorrection);
            motorRightFront.setPower(+power * direction - anglecorrection - anglecorrection);
            motorLeftBack.setPower(+power * direction + anglecorrection - anglecorrection);
            motorLeftFront.setPower(-power * direction + anglecorrection - anglecorrection);
            op.telemetry.addData("current xpos", currentPosition[0] + "current ypos", currentPosition[1]);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public void moveLeft(double distance, double power) {
        double[] currentPosition = odom.track();
        double[] target_position = {0, 0, 0};
        double correction = 0;
        double anglecorrection = 0;
        int direction = 1;
        target_position[0] = currentPosition[0] + cos(odom.getAngle() * PI / 180) * -distance;
        target_position[1] = currentPosition[1] + sin(odom.getAngle() * PI / 180) * -distance;
        target_position[2] = currentPosition[2];
        if (distance < 0) {
            direction = -1;
        }
        double difference = distance;
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.25)) {
            currentPosition = odom.track();
            difference = abs(target_position[0] - currentPosition[0]);
            correction = (currentPosition[1] - target_position[1]) * .005;//gain
            anglecorrection = (currentPosition[2] - target_position[2]) * 0.005;
            if (difference * direction < 5) {
                power *= difference / 5;
                if (power < 0.2) {
                    power = 0.3 * power;
                }
            }
            motorRightBack.setPower(-power * direction - anglecorrection - anglecorrection);
            motorRightFront.setPower(+power * direction - anglecorrection - anglecorrection);
            motorLeftBack.setPower(+power * direction + anglecorrection - anglecorrection);
            motorLeftFront.setPower(-power * direction + anglecorrection - anglecorrection);
            op.telemetry.addData("current xpos", currentPosition[0] + "current ypos", currentPosition[1]);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public void moveAngle(double x, double y, double power) {
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startAngle =odom.getAngle();
        double[] currentPosition =odom.track();
        double[] target_position = {0, 0, 0};
        double anglecorrection;
        target_position[0] = currentPosition[0] + x;
        target_position[1] = currentPosition[1] + y;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double angleInRadians = atan2(y, x) - odom.getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        try {
            //Create File
            File myFTCfile = new File("/storage/emulated/0/tmp/OdometryTest.csv");
            if (myFTCfile.createNewFile()) {
                op.telemetry.addData("moveAngleOdometry:", "File created:%S\n", "Odomeytry" );
                op.telemetry.update();
            } else {
                op.telemetry.addData("moveAngleOdometry:", "File already exists:%S\n", "Odometry");
                op.telemetry.update();
            }
            FileWriter wFTCfile = new FileWriter(myFTCfile);

            while (op.opModeIsActive() && (difference >= 1)) {
                currentPosition = odom.track();
            /*op.telemetry.addData("targetx", target_position[0]);
            op.telemetry.addData("targety",target_position[1]);
            op.telemetry.addData("angle",angleInRadians);
            op.telemetry.addData("distance",difference);
            op.telemetry.addData("power1",anglePower[0]);
            op.telemetry.addData("power2",anglePower[1]);
            op.telemetry.update();
            op.telemetry.update();*/
                if (difference < 5) {
                    power *= difference / 10;
                    if (abs(power) < 0.2) {
                        power = 0.2;
                    }
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(y, x) - currentPosition[2] * PI / 180;
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                anglecorrection = (currentPosition[2] - target_position[2]) * 0.005;
                if (difference > 10) {
                    if (abs(anglePower[1]) > abs(anglePower[0])) {
                        anglePower[1] *= abs(1 / anglePower[1]);
                        anglePower[0] *= abs(1 / anglePower[1]);
                    } else {
                        anglePower[1] *= abs(1 / anglePower[0]);
                        anglePower[0] *= abs(1 / anglePower[0]);

                    }
                }
                motorRightBack.setPower(power * anglePower[1] + anglecorrection);
                motorRightFront.setPower(power * anglePower[0] + anglecorrection);
                motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
                motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
                difference = abs(sqrt((x) * (x) + (y) * (y)));
                op.telemetry.addData("distance", difference);
                op.telemetry.update();
                //op.sleep(3000);
                //FileWriteHandle;
                wFTCfile.write(System.currentTimeMillis() + "," + String.format("%.2f",currentPosition[0]) + "," + String.format("%.2f",currentPosition[1]) + "," +
                        String.format("%.2f",currentPosition[2]) + "," +
                        String.format("%.2f",power) + "," +
                        String.format("%.2f",anglePower[0]) + "," +
                        String.format("%.2f",anglePower[1]) + "," +
                        String.format("%.2f",anglecorrection) + "," +
                        String.format("%.2f",difference) +","+"\n"+
                        String.format("%.2f",encoder[0])+"," +String.format("%.2f",encoder[1])+","+ String.format("%.2f",encoder[2])+","+String.format("%.2f",encoder[3])+","+"\n");
            }
            wFTCfile.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        turnInPlace(startAngle, 0.5);
        stopAllMotors();
    }

    @Override
    public void moveMultidirectional(double power, double angle, float rightStick) {

    }

    }
