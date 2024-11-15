package org.firstinspires.ftc.teamcode.drivetrain.tuning

import com.acmerobotics.dashboard.CustomVariableConsumer
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.AngularRampLogger
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger
import com.acmerobotics.roadrunner.ftc.DriveType
import com.acmerobotics.roadrunner.ftc.DriveType.MECANUM
import com.acmerobotics.roadrunner.ftc.DriveView
import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FeedforwardFactory
import com.acmerobotics.roadrunner.ftc.ForwardPushTest
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger
import com.acmerobotics.roadrunner.ftc.LateralPushTest
import com.acmerobotics.roadrunner.ftc.LateralRampLogger
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta.Flavor.TELEOP
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive.DriveLocalizer
import org.firstinspires.ftc.teamcode.drivetrain.ThreeDeadWheelLocalizer

object TuningOpModes {
    const val GROUP = "quickstart"
    const val DISABLED = false

    private fun metaForClass(cls: Class<out OpMode>): OpModeMeta {
        return OpModeMeta.Builder().setName(cls.getSimpleName()).setGroup(GROUP).setFlavor(TELEOP)
            .build()
    }

    @OpModeRegistrar
    fun register(manager: OpModeManager) {
        if (DISABLED) return

        val dvf = object : DriveViewFactory {
            override fun make(hardwareMap: HardwareMap): DriveView {

                val md: MecanumDrive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
                val leftEncs: MutableList<Encoder> = mutableListOf()
                val rightEncs: MutableList<Encoder> = mutableListOf()
                val parEncs: MutableList<Encoder> = mutableListOf()
                val perpEncs: MutableList<Encoder> = mutableListOf()
                if (md.localizer is DriveLocalizer) {
                    val dl: DriveLocalizer = md.localizer
                    leftEncs.add(dl.leftFront)
                    leftEncs.add(dl.leftBack)
                    rightEncs.add(dl.rightFront)
                    rightEncs.add(dl.rightBack)
                } else if (md.localizer is ThreeDeadWheelLocalizer) {
                    val dl: ThreeDeadWheelLocalizer = md.localizer
                    parEncs.add(dl.par0)
                    parEncs.add(dl.par1)
                    perpEncs.add(dl.perp)
                } else {
                    throw java.lang.RuntimeException(
                        "unknown localizer: " + md.localizer.javaClass.getName()
                    )
                }
                return DriveView(MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    hardwareMap.getAll<LynxModule>(LynxModule::class.java),
                    listOf(
                        md.leftFront, md.leftBack
                    ),
                    listOf(
                        md.rightFront, md.rightBack
                    ),
                    leftEncs,
                    rightEncs,
                    parEncs,
                    perpEncs,
                    md.lazyImu,
                    md.voltageSensor,
                    FeedforwardFactory {
                        MotorFeedforward(
                            MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick
                        )
                    })
            }
        }

        manager.register(metaForClass(AngularRampLogger::class.java), AngularRampLogger(dvf) )
        manager.register(metaForClass(ForwardPushTest::class.java ), ForwardPushTest(dvf) )
        manager.register(metaForClass(ForwardRampLogger::class.java ), ForwardRampLogger(dvf) )
        manager.register(metaForClass(LateralPushTest::class.java ), LateralPushTest(dvf) )
        manager.register(metaForClass(LateralRampLogger::class.java ), LateralRampLogger(dvf) )
        manager.register(metaForClass(ManualFeedforwardTuner::class.java ), ManualFeedforwardTuner(dvf) )
        manager.register(metaForClass(MecanumMotorDirectionDebugger::class.java ), MecanumMotorDirectionDebugger(dvf) )
        manager.register(metaForClass(DeadWheelDirectionDebugger::class.java ), DeadWheelDirectionDebugger(dvf) )
        manager.register(metaForClass(ManualFeedbackTuner::class.java ), ManualFeedbackTuner::class.java )
        manager.register(metaForClass(SplineTest::class.java ), SplineTest::class.java )
        manager.register(metaForClass(LocalizationTest::class.java ), LocalizationTest::class.java )

        FtcDashboard.getInstance()
            .withConfigRoot(CustomVariableConsumer { configRoot: CustomVariable ->
                listOf<Class<out Any>>(
                    AngularRampLogger::class.java,
                    ForwardRampLogger::class.java,
                    LateralRampLogger::class.java,
                    ManualFeedforwardTuner::class.java,
                    MecanumMotorDirectionDebugger::class.java,
                    ManualFeedbackTuner::class.java
                ).forEach {
                    configRoot.putVariable(
                        it.getSimpleName(),
                        ReflectionConfig.createVariableFromClass(it)
                    )
                }
            })
    }
}
