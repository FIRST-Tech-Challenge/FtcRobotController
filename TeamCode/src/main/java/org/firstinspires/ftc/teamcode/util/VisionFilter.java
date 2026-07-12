package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.LinkedList;
import java.util.Queue;

/**
 * VisionFilter: Smooths noisy AprilTag position readings using a moving average filter.
 *
 * What it does:
 * - Stores the last N readings (buffer size, default 8)
 * - Averages X, Y, Z coordinates separately across all buffered readings
 * - Returns a smoothed Position that reduces sensor noise
 * - Helps arm targeting system work with cleaner data
 *
 * Usage:
 *   VisionFilter filter = new VisionFilter(8); // buffer size = 8 readings
 *   filter.addReading(newRawPosition);
 *   if(filter.isBufferFull()) {
 *       Position smoothedPos = filter.getFilteredReading();
 *   }
 */
public class VisionFilter {
    private Queue<Position> readingBuffer;
    private int bufferSize;
    private static final DistanceUnit UNIT = DistanceUnit.INCH;

    /**
     * Constructor: Initialize filter with specified buffer size.
     *
     * @param bufferSize Number of readings to average (typically 5-10)
     *                   - Smaller (5): Faster response, less smoothing
     *                   - Larger (10): More smoothing, slower response
     */
    public VisionFilter(int bufferSize) {
        this.bufferSize = bufferSize;
        this.readingBuffer = new LinkedList<>();
    }

    /**
     * Add a new raw AprilTag reading to the buffer.
     * When buffer is full, oldest reading is automatically discarded.
     *
     * @param reading Raw Position from camera (x, y, z in inches)
     */
    public void addReading(Position reading) {
        if (reading == null) {
            return; // Ignore null readings
        }
        if (reading.x == 0 && reading.y == 0 && reading.z == 0) {
            return; // Ignore zero readings from missed/invalid AprilTag frames
        }

        readingBuffer.offer(reading);

        // Keep buffer at fixed size by removing oldest reading
        if (readingBuffer.size() > bufferSize) {
            readingBuffer.poll();
        }
    }

    /**
     * Check if buffer has enough readings for reliable filtering.
     * Returns false during startup until buffer fills up.
     *
     * @return true if buffer contains bufferSize readings, false otherwise
     */
    public boolean isBufferFull() {
        return readingBuffer.size() == bufferSize;
    }

    public boolean hasData() {
        return !readingBuffer.isEmpty();
    }

    /**
     * Get the current smoothed position by averaging all buffered readings.
     * Each X, Y, Z component is averaged independently.
     *
     * Example:
     *   If buffer contains: [Pos(1,2,3), Pos(3,2,1), Pos(2,2,2)]
     *   Returns: Pos((1+3+2)/3, (2+2+2)/3, (3+1+2)/3) = Pos(2, 2, 2)
     *
     * @return Averaged Position, or null if buffer is empty
     */
    public Position getFilteredReading() {
        if (readingBuffer.isEmpty()) {
            return null;
        }

        double sumX = 0;
        double sumY = 0;
        double sumZ = 0;
        int count = 0;

        // Sum all readings
        for (Position pos : readingBuffer) {
            sumX += pos.x;
            sumY += pos.y;
            sumZ += pos.z;
            count++;
        }

        // Calculate average
        double avgX = sumX / count;
        double avgY = sumY / count;
        double avgZ = sumZ / count;

        // Return as Position object (using milliseconds as 4th parameter for timestamp)
        return new Position(UNIT, avgX, avgY, avgZ, System.currentTimeMillis());
    }

    /**
     * Get the number of readings currently in the buffer.
     * Useful for debugging/telemetry.
     *
     * @return Current buffer size (0 to bufferSize)
     */
    public int getBufferCount() {
        return readingBuffer.size();
    }

    /**
     * Clear all readings from buffer (reset filter state).
     * Use this if switching between different AprilTags or debugging.
     */
    public void clearBuffer() {
        readingBuffer.clear();
    }
}
