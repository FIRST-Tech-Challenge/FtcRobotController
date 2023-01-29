//package org.firstinspires.ftc.teamcodekt.meepmeep
//
//import com.noahbres.meepmeep.MeepMeep
//import com.noahbres.meepmeep.MeepMeep.Background
//import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
//import com.noahbres.meepmeep.roadrunner.DriveShim
//import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
//import ftc.rogue.blacksmith.Anvil
//import ftc.rogue.blacksmith.meepmeep.MeepMeepPersistence
//import ftc.rogue.blacksmith.units.GlobalUnits
//import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.*
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
//import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto
//
//private val startPose = GlobalUnits.pos(-91, -163, 90)
//
//fun main() {
//    val mm = MeepMeep(800)
//
//    MeepMeepPersistence(mm).restore()
//
//    val bot = DefaultBotBuilder(mm)
//        .setColorScheme(ColorSchemeBlueDark())
//        .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
//        .setDimensions(12.0, 12.0)
//        .followTrajectorySequence(::mainTraj)
//
//    mm.setBackground(Background.FIELD_POWERPLAY_OFFICIAL)
//        .setDarkMode(true)
//        .setBackgroundAlpha(0.95f)
//        .addEntity(bot)
//        .start()
//}
//
///**
// * LOW AUTO:
// * Run a trajectory to deposit on the low pole during auto
// * - Tiernan
// */
//private fun mainTraj(drive: DriveShim) =
//    Anvil.formTrajectory(drive, startPose)
//
//        // Deposit first cone on high
//        .awaitInitialGoToDeposit()
//
//
//        // Turns while at the high deposit to not knock over cone stack when going to intake
////        .turn(180)
//
//        // Prepare for cycling by moving to the intaking position
//        .inReverse { splineToSplineHeading(-151, -42, 0, 180) }
//
//        // Turn and then intake
//        .turn(-32)
//
//        .awaitIntake()
//
//        /*
//            Loop is arranged a bit strangely - because intaking initially is a bit different, the last
//            deposit is managed automatically
//         */
//        .doTimes(RogueBaseAuto.NUM_CYCLES-1) {
//            goToDeposit(it)
//            awaitDeposit()
//
//            awaitGoToIntake(it)
//            awaitIntake()
//        }
//        // Final deposit - must be tweaked here
//        .splineTo(-151+11.8727, -42-7.4189, -32)
//        .awaitDeposit()
//
//        .build<TrajectorySequence>()
//
//private fun Anvil.awaitInitialGoToDeposit() = this
//    .splineTo(-80, -75, 44)
//    .awaitDeposit()
//    .back(10)
//    .turn(90-44)
//    .splineToSplineHeading(-80, -17.5, 50, 50)
//
//private fun Anvil.goToDeposit(it: Int) = when (it) {
//    /*
//        The offset values are from sin(32) and cos(32) degrees.
//        Used to spline in a straight line. This is advantageous to maintain localization better.
//    */
//    0 -> splineTo(-151+11.8727, -42-7.4189, -32)
//    1 -> splineTo(-151+11.8727, -42-7.4189, -32)
//    2 -> splineTo(-151+11.8727, -42-7.4189, -32)
//    3 -> splineTo(-151+11.8727, -42-7.4189, -32)
//
//    else -> noop
//}
//
//private fun Anvil.awaitGoToIntake(it: Int) = when (it) {
//
//    0 -> splineTo(-151, -42, 148)
//    1 -> splineTo(-151, -42, 148)
//    2 -> splineTo(-151, -42, 148)
//    3 -> splineTo(-151, -42, 148)
//    else -> noop
//}.doInReverse()
//
//private fun Anvil.awaitDeposit() = this
//    // Extra time allowed for depositing - NO REASON TO RUSH IT
//    .waitTime(350)
//
//private fun Anvil.awaitIntake() = this
//    // Extra time allowed for intaking - NO REASON TO RUSH IT
//    .waitTime(350)
