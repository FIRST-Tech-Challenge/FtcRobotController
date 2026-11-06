package org.firstinspires.ftc.teamcode.opmode

import com.pedropathing.algorithm.Foresight
import com.pedropathing.follower.Follower
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystem.Brakes
import org.firstinspires.ftc.teamcode.subsystem.Deflowerer
import org.firstinspires.ftc.teamcode.subsystem.Gate
import org.firstinspires.ftc.teamcode.subsystem.Intake
import org.firstinspires.ftc.teamcode.subsystem.Limelight
import org.firstinspires.ftc.teamcode.subsystem.Outtake
import org.firstinspires.ftc.teamcode.subsystem.Shooter
import org.firstinspires.ftc.teamcode.subsystem.TaskManager
import org.firstinspires.ftc.teamcode.subsystem.turret.PTurret

abstract class FullOpMode : OpMode() {
    val taskManager: TaskManager = TaskManager()
    val follower: Follower by lazy {
        with (Constants) {
            Follower(
                PinpointLocalizer(hardwareMap, pinpointConfig),
                Mecanum(hardwareMap, mecanumConfig),
                Foresight(foresightConfig)
            ).apply {
                taskManager.loop.init += { this.update() }
            }
        }
    }
    val shooter: Shooter by lazy {
        Shooter(
            taskManager = taskManager,
            follower = follower,
            outtake = Outtake(
                motorLeft = hardwareMap["outtakeLeft"] as DcMotorEx,
                motorRight = hardwareMap["outtakeRight"] as DcMotorEx,
            ),
            turret = PTurret(
                servoLeft = hardwareMap["turretLeft"] as Servo,
                servoMid = hardwareMap["turretMid"] as Servo,
                servoRight = hardwareMap["turretRight"] as Servo,
            ),
            gate = Gate(
                servo = hardwareMap["gate"] as Servo
            )
        )
    }
    val intake: Intake by lazy {
        Intake(
            intakeMotorLeft = hardwareMap["intakeLeft"] as DcMotorEx,
            intakeMotorRight = hardwareMap["intakeRight"] as DcMotorEx
        )
    }
    val brakes: Brakes by lazy {
        Brakes(
            taskManager = taskManager,
            follower = follower,
            servoLeft = hardwareMap["brakesLeft"] as Servo,
            servoRight = hardwareMap["brakesRight"] as Servo,
        )
    }
    val deflowerer: Deflowerer by lazy {
        Deflowerer(
            servoLeft = hardwareMap["deflowererLeft"] as Servo,
            servoRight = hardwareMap["deflowererRight"] as Servo,
        )
    }
    val limelight: Limelight by lazy {
        Limelight(
            taskManager = taskManager,
            limelight3A = hardwareMap["limelight"] as Limelight3A
        )
    }

    override fun init_loop() {
        taskManager.initLoop()
    }

    override fun loop() {
        taskManager.loop()
    }
}