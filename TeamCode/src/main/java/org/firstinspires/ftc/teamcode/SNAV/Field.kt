package org.firstinspires.ftc.teamcode.SNAV

class Field(val xSize: Int, val ySize: Int) {
    var field: MutableList<MutableList<Int>> = MutableList(ySize) { MutableList(xSize) {0} }

    fun isTraversable(point: Point): Boolean {
        if (point.y < 0 || point.y > field.size - 1) return false
        if (point.x < 0 || point.x > field[0].size - 1) return false
        return field[point.y][point.x] == 0;
    }

    fun getPath(start: PathPoint, end: PathPoint): MutableList<PathPoint>? {
        var finished: Boolean = false;
        var used: MutableList<PathPoint> = mutableListOf()
        used.add(start)
        while (!finished) {
            var newOpen: MutableList<PathPoint> = mutableListOf()
            for (i in used.indices) {
                var point: PathPoint = used[i]
                for (neighbor: PathPoint in point.generateNeighbors()) {
                    if (!used.contains(neighbor) && !newOpen.contains(neighbor)) {
                        newOpen.add(neighbor)
                    }
                }
            }

            for (point: PathPoint in newOpen) {
                used.add(point);
                if (end.equals(point)) {
                    finished = true
                    break
                }
            }

            if (!finished && newOpen.isEmpty()) {
                return null
            }
        }

        // Assemble the path
        val path: MutableList<PathPoint> = mutableListOf()
        var point: PathPoint = used[used.size - 1]
        while (point.previous != null) {
            path.add(0, point)
            point = point.previous!!
        }

        return path
    }
}