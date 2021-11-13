package org.firstinspires.ftc.teamcode.Util.CoordinateSystem.PathFinder;

import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.Coordinate;
import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.Field;
import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.Path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AStarPathFinder {
    private final List<Node> open;
    private final List<Node> closed;
    private final Path path;
    private final Field field;
    private final boolean diagonal;
    private Node now;
    private Coordinate start;
    private Coordinate end;

    AStarPathFinder(Field field, boolean diagonal) {
        this.open = new ArrayList<>();
        this.closed = new ArrayList<>();
        this.path = new Path();
        this.field = field;
        this.diagonal = diagonal;
    }

    /**
     * * Looks in a given List for a node
     * *
     * * @return (bool) NeighborInListFound
     */
    private static boolean findNeighborInList(List<Node> array, Node node) {
        return array.stream().anyMatch((n) -> (n.getX() == node.getX() && n.getY() == node.getY()));
    }

    public static void main(String[] args) {
        // -1 = blocked
        // 0+ = additional movement cost
        Field field = new Field();
        AStarPathFinder as = new AStarPathFinder(field, true);
        Path path = as.getShortestPath(new Coordinate(0, 0), new Coordinate(7, 7));
        if (path != null) {
            path.getPath().forEach((n) -> {
                System.out.print("[" + n.getX() + ", " + n.getY() + "] ");
                field.set(new Coordinate(n.getX(), n.getY()), -1);
            });
            System.out.printf("\nTotal cost: %.02f\n", path.get(path.length() - 1).getG());

            for (int[] fieldRow : field.getField()) {
                for (int vertex : fieldRow) {
                    switch (vertex) {
                        case 0:
                            System.out.print("_");
                            break;
                        case -1:
                            System.out.print("*");
                            break;
                        default:
                            System.out.print("#");
                    }
                }
                System.out.println();
            }
        }
    }

    /**
     * Finds path to xEnd/yEnd or returns null
     *
     * @param start the starting coordinate
     * @param end   coordinates of the target position
     * @return the path
     */
    public Path getShortestPath(Coordinate start, Coordinate end) {
        this.start = start;
        this.now = new Node(null, start.getX(), start.getY(), 0, 0);
        this.end = end;
        this.closed.add(this.now);
        addNeighborsToOpenList();
        while (this.now.getX() != this.end.getX() || this.now.getY() != this.end.getY()) {
            if (this.open.isEmpty()) { // Nothing to examine
                return null;
            }
            this.now = this.open.get(0); // get first node (lowest f score)
            this.open.remove(0); // remove it
            this.closed.add(this.now); // and add to the closed
            addNeighborsToOpenList();
        }
        this.path.add(0, this.now);
        while (this.now.getX() != this.start.getX() || this.now.getY() != this.start.getY()) {
            this.now = this.now.parent;
            this.path.add(0, this.now);
        }
        return this.path;
    }

    /**
     * * Calculate distance between this.now and xEnd/yEnd
     * *
     * * @return (int) distance
     */
    private double distance(int dx, int dy) {
        if (this.diagonal) { // if diagonal movement is allowed
            return Math.hypot(this.now.getX() + dx - this.end.getX(), this.now.getY() + dy - this.end.getY()); // return the hypotenuse
        } else {
            return Math.abs(this.now.getX() + dx - this.end.getX()) + Math.abs(this.now.getY() + dy - this.end.getY()); // return the "Manhattan distance"
        }
    }

    private void addNeighborsToOpenList() {
        Node node;
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                if (!this.diagonal && x != 0 && y != 0) {
                    continue; // skip if diagonal movement is not allowed
                }
                node = new Node(this.now, this.now.getY() + x, this.now.getY() + y, this.now.getG(), this.distance(x, y));
                if ((x != 0 || y != 0) // not this.now
                        && this.now.getX() + x >= 0 && this.now.getX() + x < this.field.length // check maze boundaries
                        && this.now.getY() + y >= 0 && this.now.getY() + y < this.field.width
                        && this.field.isBlocked(new Coordinate(this.now.getY() + y, this.now.getX() + x)) // check if square is walkable
                        && !findNeighborInList(this.open, node) && !findNeighborInList(this.closed, node)) { // if not already done
                    node.setG(node.parent.getG() + 1.); // Horizontal/vertical cost = 1.0
                    node.setG(node.getG() + field.get(new Coordinate(this.now.getY() + y, this.now.getX() + x))); // add movement cost for this square

                    // diagonal cost = sqrt(hor_cost² + vert_cost²)
                    // in this example the cost would be 12.2 instead of 11
                        /*
                        if (diag && x != 0 && y != 0) {
                            node.g += .4;	// Diagonal movement cost = 1.4
                        }
                        */
                    this.open.add(node);
                }
            }
        }
        Collections.sort(this.open);
    }
}
