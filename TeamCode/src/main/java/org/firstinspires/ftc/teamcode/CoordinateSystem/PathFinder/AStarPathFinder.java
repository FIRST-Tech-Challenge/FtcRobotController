package org.firstinspires.ftc.teamcode.CoordinateSystem.PathFinder;

import org.firstinspires.ftc.teamcode.CoordinateSystem.Coordinate;
import org.firstinspires.ftc.teamcode.CoordinateSystem.Field;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;

class AStarPathFinder {
    private final List<Node> open;
    private final List<Node> closed;
    private final List<Node> path;
    private final Field field;
    private Node now;
    private final int xStart;
    private final int yStart;
    private int xEnd, yEnd;
    private final boolean diagonal;

    // Node class for convenience
    static class Node implements Comparable {
        public Node parent;
        public int x, y;
        public double g;
        public double h;
        Node(Node parent, int xpos, int ypos, double g, double h) {
            this.parent = parent;
            this.x = xpos;
            this.y = ypos;
            this.g = g;
            this.h = h;
        }
        // Compare by f value (g + h)
        @Override
        public int compareTo(Object o) {
            Node that = (Node) o;
            return (int)((this.g + this.h) - (that.g + that.h));
        }
    }

    AStarPathFinder(Field field, int xstart, int ystart, boolean diag) {
        this.open = new ArrayList<>();
        this.closed = new ArrayList<>();
        this.path = new ArrayList<>();
        this.field = field;
        this.now = new Node(null, xstart, ystart, 0, 0);
        this.xStart = xstart;
        this.yStart = ystart;
        this.diagonal = diag;
    }
    /*
     ** Finds path to xEnd/yEnd or returns null
     **
     ** @param (int) xEnd coordinates of the target position
     ** @param (int) yEnd
     ** @return (List<Node> | null) the path
     */
    public List<Node> findPathTo(int xEnd, int yEnd) {
        this.xEnd = xEnd;
        this.yEnd = yEnd;
        this.closed.add(this.now);
        addNeighborsToOpenList();
        while (this.now.x != this.xEnd || this.now.y != this.yEnd) {
            if (this.open.isEmpty()) { // Nothing to examine
                return null;
            }
            this.now = this.open.get(0); // get first node (lowest f score)
            this.open.remove(0); // remove it
            this.closed.add(this.now); // and add to the closed
            addNeighborsToOpenList();
        }
        this.path.add(0, this.now);
        while (this.now.x != this.xStart || this.now.y != this.yStart) {
            this.now = this.now.parent;
            this.path.add(0, this.now);
        }
        return this.path;
    }
    /*
     ** Looks in a given List<> for a node
     **
     ** @return (bool) NeightborInListFound
     */
    private static boolean findNeighborInList(List<Node> array, Node node) {
        return array.stream().anyMatch((n) -> (n.x == node.x && n.y == node.y));
    }
    /*
     ** Calulate distance between this.now and xend/yend
     **
     ** @return (int) distance
     */
    private double distance(int dx, int dy) {
        if (this.diagonal) { // if diagonal movement is allowed
            return Math.hypot(this.now.x + dx - this.xEnd, this.now.y + dy - this.yEnd); // return hypotenuse
        } else {
            return Math.abs(this.now.x + dx - this.xEnd) + Math.abs(this.now.y + dy - this.yEnd); // else return "Manhattan distance"
        }
    }
    private void addNeighborsToOpenList() {
        Node node;
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                if (!this.diagonal && x != 0 && y != 0) {
                    continue; // skip if diagonal movement is not allowed
                }
                node = new Node(this.now, this.now.x + x, this.now.y + y, this.now.g, this.distance(x, y));
                if ((x != 0 || y != 0) // not this.now
                        && this.now.x + x >= 0 && this.now.x + x < this.field.length // check maze boundaries
                        && this.now.y + y >= 0 && this.now.y + y < this.field.width
                        && this.field.isBlocked( new Coordinate(this.now.y + y, this.now.x + x)) // check if square is walkable
                        && !findNeighborInList(this.open, node) && !findNeighborInList(this.closed, node)) { // if not already done
                    node.g = node.parent.g + 1.; // Horizontal/vertical cost = 1.0
                    node.g += field.get(new Coordinate(this.now.y + y, this.now.x + x)); // add movement cost for this square

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

    public static void main(String[] args) {
        // -1 = blocked
        // 0+ = additional movement cost
        Field field = new Field();
        AStarPathFinder as = new AStarPathFinder(field, 0, 0, true);
        List<Node> path = as.findPathTo(7, 7);
        if (path != null) {
            path.forEach((n) -> {
                System.out.print("[" + n.x + ", " + n.y + "] ");
                field.set(new Coordinate(n.x, n.y), -1);
            });
            System.out.printf("\nTotal cost: %.02f\n", path.get(path.size() - 1).g);

            for (int[] fieldRow : field.getField()) {
                for (int maze_entry : fieldRow) {
                    switch (maze_entry) {
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
}
