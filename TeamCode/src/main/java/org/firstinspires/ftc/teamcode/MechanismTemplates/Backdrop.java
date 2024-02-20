import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

class Pixel {
    private String identity;

    public Pixel(String color) {
        this.identity = color;
    }

    public String identity() {
        return this.identity;
    }

    @Override
    public String toString() {
        if (identity == null || identity.isEmpty()) {
            return "\u2B21"; // Default to a white hexagon (change this character if needed)
        }
        Map<String, String> colors = new HashMap<>();
        colors.put("white", "\u2B22");  // White hexagon with white foreground color
        colors.put("yellow", "\u001B[93m\u2B22\u001B[0m");  // White hexagon with yellow foreground color
        colors.put("purple", "\u001B[95m\u2B22\u001B[0m");  // White hexagon with purple foreground color
        colors.put("green", "\u001B[92m\u2B22\u001B[0m");  // White hexagon with green foreground color
        return colors.getOrDefault(identity, "");  // Default to white hexagon
    }
}

class Backdrop {
  Telementry tel;
    private List<List<Pixel>> backdrop;
    private List<Integer> legalMoves;

    public Backdrop(Telementry tel) {
        this.tel = tel;
        backdrop = new ArrayList<>();
        legalMoves = new ArrayList<>();
        for (int row = 0; row < 12; row++) {
            backdrop.add(new ArrayList<>());
            for (int col = 0; col < (row % 2 == 0 ? 6 : 7); col++) {
                backdrop.get(row).add(new Pixel(null));
            }
        }
    }

    public boolean checkInBounds(int row, int col) {
        if (row > 11) {
            tel.addLine("Row must be between 0 and 11");
        }
        if (col > 6) {
            tel.addLine("Column must be between 0 and 6");
        }
        if (row % 2 == 0 && col > 5) {
            tel.addLine("Column must be between 0 and 5 for even rows");
        }
        if (row % 2 == 1 && col > 6) {
            tel.addLine("Column must be between 0 and 6 for odd rows");
        }else{
        return true;
        }
      return false;
    }

    public boolean isValidMosaicPixel(int row, int col) {
        checkInBounds(row, col);
        return backdrop.get(row).get(col).identity() != null && !backdrop.get(row).get(col).identity().equals("white");
    }

    public int calculateScore(boolean verbose) {
        int score = 0;
        boolean[] heightBonuses = new boolean[3];
        for (int row = 0; row < 12; row++) {
            for (int col = 0; col < (row % 2 == 0 ? 6 : 7); col++) {
                if (backdrop.get(row).get(col).identity() == null) {
                    continue;
                }
                score += 5;

                if (row >= 2) {
                    heightBonuses[0] = true;
                }
                if (row >= 5) {
                    heightBonuses[1] = true;
                }
                if (row >= 8) {
                    heightBonuses[2] = true;
                }

                if (row % 2 == 0) {
                    if (isValidMosaicPixel(row, col) && isValidMosaicPixel(row - 1, col)
                            && isValidMosaicPixel(row - 1, col + 1)) {
                        String pixelA = backdrop.get(row).get(col).identity();
                        String pixelB = backdrop.get(row - 1).get(col).identity();
                        String pixelC = backdrop.get(row - 1).get(col + 1).identity();
                    } else {
                        continue;
                    }
                } else {
                    if (col == 0 || col == 6) {
                        continue;
                    }
                    if (isValidMosaicPixel(row, col) && isValidMosaicPixel(row - 1, col)
                            && isValidMosaicPixel(row - 1, col - 1)) {
                        String pixelA = backdrop.get(row).get(col).identity();
                        String pixelB = backdrop.get(row - 1).get(col).identity();
                        String pixelC = backdrop.get(row - 1).get(col + 1).identity();
                    } else {
                        continue;
                    }
                }

                if (!(pixelA.equals(pixelB) && pixelB.equals(pixelC) || !pixelA.equals(pixelB) && !pixelB.equals(pixelC) && !pixelC.equals(pixelA))) {
                    continue;
                }

                String[] acceptableEdges = {"white", "board", null};
                boolean[] borderIdentities = {
                    backdrop.get(row).get(col + 1).identity().equalsAny(acceptableEdges),
                    backdrop.get(row - 1).get(col + 2).identity().equalsAny(acceptableEdges),
                    backdrop.get(row - 2).get(col + 1).identity().equalsAny(acceptableEdges),

                    backdrop.get(row - 2).get(col).identity().equalsAny(acceptableEdges),
                    backdrop.get(row - 2).get(col - 1).identity().equalsAny(acceptableEdges),
                    backdrop.get(row - 1).get(col - 2).identity().equalsAny(acceptableEdges),

                    backdrop.get(row).get(col - 1).identity().equalsAny(acceptableEdges),
                    backdrop.get(row + 1).get(col).identity().equalsAny(acceptableEdges),
                    backdrop.get(row + 1).get(col + 1).identity().equalsAny(acceptableEdges)
                };

                if (!containsFalse(borderIdentities)) {
                    if (verbose) {
                        System.out.println("Mosaic found at (" + row + ", " + col + ")!");
                    }
                    score += 10;
                }
                if (backdrop.get(row).get(col).identity().equals(backdrop.get(row).get(col + 1).identity())
                        && backdrop.get(row).get(col).identity().equals(backdrop.get(row + 1).get(col).identity())) {
                    score += 10;
                }
            }
        }

        for (boolean bonus : heightBonuses) {
            if (bonus) {
                score += 10;
            }
        }

        return score;
    }

    public boolean isLegalMove(int row, int col) {
        checkInBounds(row, col);

        if (backdrop.get(row).get(col).identity() != null) {
            return false;
        }

        if (row % 2 != 0) {
            Pixel pixelLeft = col > 0 ? backdrop.get(row - 1).get(col - 1) : new Pixel("board");
            Pixel pixelRight = col < 6 ? backdrop.get(row - 1).get(col) : new Pixel("board");

            if (pixelLeft.identity() == null || pixelRight.identity() == null) {
                return false;
            }
        } else {
            if (row != 0) {
                Pixel pixelLeft = backdrop.get(row - 1
