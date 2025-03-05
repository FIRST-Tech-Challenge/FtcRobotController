package org.firstinspires.ftc.teamcode.drive.actuators;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AutoAlignWithIMU {
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private IMU imu;  // Instância da IMU BHI260AP
    private final double kP = 0.05; // Constantes PID
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double previousError = 0;
    private double integral = 0;
    private ElapsedTime runtime = new ElapsedTime();

    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        // Configuração do IMU (BHI260AP) pode ser feita aqui
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    }

    public void alignToTarget(double targetAngle) {
        resetEncoders();

        double currentAngle = getIMUOrientation(); // Obtém a orientação atual do robô a partir da IMU

        while (Math.abs(currentAngle - targetAngle) > 2) { // Defina o erro aceitável
            double error = targetAngle - currentAngle;
            double correction = getPIDControl(error);

            // Ajusta a rotação para alinhar o robô
            turnRobot(correction);

            currentAngle = getIMUOrientation(); // Atualiza a orientação
        }

        stopMotors(); // Para o robô após atingir a orientação desejada
    }

    public double getPIDControl(double error) {
        integral += error * runtime.time();
        double derivative = (error - previousError) / runtime.time();

        double output = kP * error + kI * integral + kD * derivative;

        previousError = error;
        runtime.reset();

        return output;
    }

    public void turnRobot(double correction) {
        // Usa a correção do PID para ajustar a rotação
        leftFront.setPower(-correction);  // Motor da frente esquerda (gira no sentido desejado)
        leftRear.setPower(-correction);   // Motor traseiro esquerdo
        rightFront.setPower(correction);  // Motor da frente direita
        rightRear.setPower(correction);   // Motor traseiro direito
    }

    public void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getIMUOrientation() {
        // Obtém a orientação do robô usando a IMU (BHI260AP)
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();  // Lê a orientação do IMU

        // Retorna o valor do primeiro ângulo (geralmente usado para a rotação em torno do eixo Z)
        double currentAngle = angles.getYaw(AngleUnit.RADIANS);

        return currentAngle;
    }
}
