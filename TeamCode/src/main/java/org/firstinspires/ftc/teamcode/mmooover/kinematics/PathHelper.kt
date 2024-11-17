package org.firstinspires.ftc.teamcode.mmooover.kinematics

class PathHelper internal constructor() {
    data class LinearMoveCommand(
        val x: Double,
        val y: Double,
        val r: Double
    ): AuthoringCommand

    val Number.deg get() = Math.toRadians(this.toDouble())

    val commands: MutableList<AuthoringCommand> = mutableListOf()

    /**
     * Automatically insert matching XYR commands when starting a **new motion block after the first**.
     *
     * This means that you don't have to re-enter the last position and rotation keyframe after
     * using [run]/[launch]/[await].
     */
    var autoInsertHolds = true

    fun r(r: Number) = commands.add(RImpl(r.toDouble()))
    fun m(x: Number, y: Number) = commands.add(XYImpl(x.toDouble(), y.toDouble()))
    fun m(x: Number, y: Number, r: Number) = commands.add(XYRImpl(x.toDouble(), y.toDouble(), r.toDouble()))
    fun line(x: Number, y: Number, r: Number) = commands.add(LinearMoveCommand(x.toDouble(), y.toDouble(), r.toDouble()))
    fun run(target: String) = commands.add(RunImpl(target))
    fun launch(target: String) = commands.add(RunAsyncImpl(target))
    fun await(target: String) = commands.add(AwaitImpl(target))

    fun generate2(): List<BytecodeUnit> {
        // break into 'move segments' and 'function segments'.
        // true: move
        // false: function
        val segments: MutableList<Pair<Boolean, MutableList<AuthoringCommand>>> = mutableListOf()
        var cursor = 0
        while (cursor < commands.size) {
            // Peek the first element to determine the type
            val segment: MutableList<AuthoringCommand> = mutableListOf()
            var command = commands[cursor]
            val mode = command is MotionCommand
            // Repeat while that type continues to match
            if (mode) {
                while (command is MotionCommand) {
                    segment.add(command)
                    if (++cursor >= commands.size) break
                    command = commands[cursor]
                }
                segments.add(Pair(true, segment))
            }
            else {
                segment.add(command)
                cursor++
            }
            // Pack and store
            segments.add(Pair(mode, segment))
        }
        val units: MutableList<BytecodeUnit> = mutableListOf()
        // compile segments
        var i = 0
        var autoInsertSlot: XYRCommand? = null
        for ((mode, segment) in segments) {
            if (mode) {
                // technically a noop but makes typechecker happy
                val typedSegment = segment.filterIsInstance<MotionCommand>().toMutableList()
                assert(typedSegment.isNotEmpty()) { "Empty motion block, somehow" }
                if (autoInsertHolds && autoInsertSlot != null) {
                    val first = typedSegment[0]
                    if (!autoInsertSlot.approx(first)) {
                        typedSegment.add(0, autoInsertSlot)
                    }
                }
                val generatedBytecode = generateMovePart(typedSegment)
                units.addAll(generatedBytecode)
                autoInsertSlot = generatedBytecode.last()
            } else {
                for (part in segment) {
                    when (part) {
                        is BytecodeUnit -> units.add(part)
                        is LinearMoveCommand -> units.add(MoveImpl(part.x, part.y, part.r))
                        else -> throw IllegalStateException("Not sure what to do with ${part::class.simpleName} here...")
                    }
                }
            }
            i++
        }
        return units
    }

    fun generateMovePart(moveCommands: List<MotionCommand>): List<MoveCommand> {
        assert(moveCommands.size >= 2) { "Not enough keyframes (need at least 2, have ${moveCommands.size})" }
        // Assert that the first and last keyframes are at least XY
        assert(moveCommands[0] is XYCommand) { "First keyframe must have X/Y data" }
        assert(moveCommands.last() is XYCommand) { "Last keyframe must have X/Y data" }

        val xyCommands: MutableList<XYCommand> = mutableListOf()
        val mapping: MutableList<Int> = mutableListOf()
        val isExact: MutableList<Boolean> = mutableListOf()
        val fractionals: MutableList<Double> = mutableListOf()

        var mapI = -1
        var splitCount = -1
        for (kf in moveCommands) {
            if (kf is XYCommand) {
                mapping.add(++mapI)
                isExact.add(true)
                xyCommands.add(kf)
                fractionals.add(0.0)
                if (splitCount > 0) {
                    val onePart = 1.0 / (splitCount + 1)
                    for (i in 1 .. splitCount) {
                        fractionals[fractionals.size - i - 1] = 1 - (onePart * i)
                    }
                }
                splitCount = 0
            } else {
                mapping.add(mapI)
                isExact.add(false)
                fractionals.add(0.0)
                splitCount++
            }
        }

        // Generate the X/Y curves
        val xyCurves = CubicSplineSolver.solve2DMultiSegment(
            xyCommands.map { it.x }.toDoubleArray(),
            xyCommands.map { it.y }.toDoubleArray()
        )
        val timings = buildList {
            for (item in xyCurves) add(item.offset + item.tFrom)
            val last = xyCurves.last()
            add(last.offset + last.tTo)
        }
        val timing: MutableList<Double> = mutableListOf()
        val rotValues: MutableList<Double> = mutableListOf()

        for ((i, key) in moveCommands.withIndex()) {
            if (key !is RCommand) continue
            if (isExact[i]) {
                timing.add(timings[mapping[i]])
            } else {
                val span = timings[mapping[i]] - timings[mapping[i] - 1]
                val currentT = timings[mapping[i]-1] + span * fractionals[i]
                timing.add(currentT)
            }
            rotValues.add(key.r)
        }

        // Generate the rotation curves
        val rotCurves = CubicSplineSolver.solveMultiSegment(
            timing.toDoubleArray(),
            rotValues.toDoubleArray()
        )

        // Generate XY waypoints
        val waypointList: MutableList<CubicSplinePair.PointData> = mutableListOf()
        for (i in xyCurves) {
            if (!waypointList.isEmpty()) waypointList.removeAt(waypointList.lastIndex) // pop the duplicate waypoint
            waypointList.addAll(i.computeWaypoints())
        }

        // zip
        val zipped = CubicSplineSolver.thirdChannel(waypointList, rotCurves)
        return zipped
    }
}

fun path(configure: PathHelper.() -> Unit): PathHelper {
    val target = PathHelper()
    target.configure()
    return target
}