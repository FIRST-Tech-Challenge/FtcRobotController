package org.firstinspires.ftc.teamcode.koawalib.commands.subsystems

import com.asiankoala.koawalib.command.commands.Cmd
import com.asiankoala.koawalib.gamepad.functionality.Stick
import com.asiankoala.koawalib.math.NVector
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.subsystem.drive.KMecanumDrive
import org.firstinspires.ftc.teamcode.koawalib.DriveHack
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign

/**
 * TeleOp drive control command
 * vector drive power is calculated with the function:
 * f(x) = max(0, s * x * (kx^3 - k + 1)) * sgn(x)
 * e.g. xPower = max(0, xScalar * leftStick.x * (xCubic * leftStick.x ^ 3 - xCubic + 1)
 * see the desmos graph for an understanding of it
 * @see <a href="https://www.desmos.com/calculator/r8hanh49bk">https://www.desmos.com/calculator/r8hanh49bk</a>
 * If not using field centric drive, leave everything after rScalar as default
 *
 * @param drive KMecanumDrive reference
 * @param leftStick left gamepad joystick
 * @param rightStick right gamepad joystick
 * @param xCubic x power k constant in scaling function
 * @param yCubic y power k constant in scaling function
 * @param rCubic r power k constant in scaling function
 * @param xScalar x scalar in scaling function
 * @param yScalar y scalar in scaling function
 * @param rScalar r scalar in scaling function
 * @param alliance robot's alliance for match
 * @param isTranslationFieldCentric translation field centric
 * @param isHeadingFieldCentric heading field centric
 * @param heading heading supplier
 * @param fieldCentricHeadingScalar angle to start deccel for field centric heading
 */
class DriveHackCmd(
    private val drive: KMecanumDrive,
    private val leftStick: Stick,
    private val rightStick: Stick,
    private val isGliding: () -> Boolean,
    private val isAimbotting: () -> Boolean,
    ps: () -> Pose,
    R: Double = 6.0,
    N: Double = 1.0,
    H: Double = 45.0.radians,
    private val W: Double = 0.8,
    private val scalars: NVector = NVector(3) { 1.0 },
    private val cubics: NVector = NVector(3) { 1.0 },
) : Cmd() {
    private val driveHack = DriveHack(ps, R, N, H)
    private fun joystickFunction(s: Double, k: Double, x: Double): Double {
        return max(0.0, s * x * (k * x.pow(3) - k + 1)) * x.sign
    }

    private fun processPowers(): Pose {
        val raws = NVector(
            leftStick.xSupplier.invoke(),
            -leftStick.ySupplier.invoke(),
            -rightStick.xSupplier.invoke()
        )
        val pre = raws mapIndexed { i, r -> joystickFunction(scalars[i], cubics[i], r) }
        var output = pre restrict 2
        if(isGliding.invoke()) output = output * W + driveHack.spaceglide((raws restrict 2).as2dVec) * (1.0 - W)
        output push if(isAimbotting.invoke()) driveHack.aimbot((raws restrict 2).as2dVec) else pre[3]
        return output.asPose
    }

    override fun execute() {
        drive.powers = processPowers()
    }

    override val isFinished: Boolean
        get() = false

    init {
        addRequirements(drive)
    }
}