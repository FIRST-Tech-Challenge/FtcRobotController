package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import dev.aether.collaborative_multitasking.ITask
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.ext.Pause
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking
import org.firstinspires.ftc.teamcode.mmooover.Pose
import org.firstinspires.ftc.teamcode.mmooover.Ramps
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power
import org.firstinspires.ftc.teamcode.mmooover.tasks.MoveToTask

@Autonomous(name = "Box Test", group = "Utilities")
class BoxTest : LinearOpMode() {

    lateinit var scheduler: MultitaskScheduler
    lateinit var hardware: Hardware
    lateinit var tracker: EncoderTracking
    lateinit var loopTimer: LoopStopwatch
    val speed2Power = Speed2Power(0.2)
    val ramps = Ramps(
        Ramps.linear(2.0),
        Ramps.linear(1 / 12.0),
        Ramps.LimitMode.SCALE
    )

    fun moveTo(target: Pose): MoveToTask {
        return MoveToTask(
            scheduler, hardware, target, tracker, loopTimer, speed2Power, ramps, telemetry
        )
    }

    fun wait(seconds: Number): ITask {
        return Pause(scheduler, seconds.toDouble())
    }

    private val Number.deg: Double; get() = Math.toRadians(this.toDouble())
    private fun hardwareInit() {
        hardware.backLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        hardware.frontLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        hardware.backRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        hardware.frontRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        hardware.clawFlip.position = Hardware.FLIP_UP;
        hardware.clawFront.position = Hardware.FRONT_OPEN;

        hardware.arm.targetPosition = 0;
        hardware.arm.mode = DcMotor.RunMode.RUN_TO_POSITION;
        hardware.arm.power = 0.3;
        hardware.wrist.position = 0.28;
        hardware.claw.position = Hardware.CLAW_CLOSE;

        // we don't have the proxy object to handle this for us
        // so manually implement the inversion
        hardware.horizontalSlide.position = Hardware.RIGHT_SLIDE_IN;
        hardware.horizontalLeft.position = 1.05 - Hardware.RIGHT_SLIDE_IN;

        hardware.lightLeft.position = Hardware.LAMP_PURPLE;
        hardware.lightRight.position = Hardware.LAMP_PURPLE;
    }

    override fun runOpMode() {
        scheduler = MultitaskScheduler()
        hardware = Hardware(hardwareMap)
        tracker = EncoderTracking(hardware)
        loopTimer = LoopStopwatch()

        scheduler.add(wait(.1))
            .then(moveTo(Pose(24.0, 0.0, 0.0)))
            .then(wait(.5))
            .then(moveTo(Pose(24.0, 24.0, 90.deg)))
            .then(wait(.5))
            .then(moveTo(Pose(0.0, 24.0, 180.deg)))
            .then(wait(.5))
            .then(moveTo(Pose(0.0, 0.0, 270.deg)))

        hardwareInit()

        waitForStart()

        while (opModeIsActive()) {
            tracker.step()
            loopTimer.click()
            scheduler.tick()
            scheduler.displayStatus(true, true, telemetry::addLine)
            telemetry.update()
        }
    }
}