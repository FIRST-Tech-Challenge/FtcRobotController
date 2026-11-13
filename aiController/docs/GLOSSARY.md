# Vision nomenclature / Nomenclature de vision

Use these words consistently in code, tests, and engineering documentation.

| Term (English / Francais) | Precise meaning |
| --- | --- |
| **Frame / image** | One image acquired from the camera. |
| **Pixel / pixel** | An image sample at position `(x, y)`; here `(0, 0)` is the top-left. |
| **Resolution / resolution** | Pixel dimensions, e.g. `640 x 480`. |
| **Region of interest (ROI) / zone d'interet** | A subset of the image examined by an algorithm. |
| **Color range / plage de couleur** | Channel limits used to identify similarly colored pixels. |
| **Mask / masque** | Image marking pixels which pass a filtering condition. |
| **Blob / region connexe** | Group of connected pixels matched by a color test. |
| **Contour / contour** | Boundary of a connected region. |
| **Bounding box / boite englobante** | Rectangle surrounding a detected object. |
| **Center `(cx, cy)` / centre** | Geometric center in image pixels, not a field-coordinate position. |
| **Area / aire** | Number of pixels inside a detected blob's contour (`px^2`). |
| **Detection / detection** | Localizing an object, often with a box and class label. |
| **Classification / classification** | Predicting what class an image/object belongs to. |
| **Label / etiquette** | Human-readable class name such as `pollen` or `nectar`. |
| **Class ID / identifiant de classe** | Integer used to represent a class; mapping belongs in the dataset or sensor setup. |
| **Confidence / score de confiance** | Model score for a class/detection. NOT returned by the simple color-blob algorithm. |
| **Inference / inference** | Running an already trained model on a new frame. |
| **Training / entrainement** | Updating a model using labeled examples; happens on a PC in this starter. |
| **Validation set / donnees de validation** | Examples used while tuning a model, separate from training. |
| **Test set / donnees de test** | Held-out examples used for final performance evaluation. |
| **False positive / faux positif** | A detection where the desired target is not actually present. |
| **False negative / faux negatif** | A real target missed by the detector. |
| **Latency / latence** | Time between image capture and usable recognition result. |
| **AprilTag / marqueur AprilTag** | Fiducial marker used to estimate identity and (when configured) pose. Not a neural classifier. |
| **Pose / pose** | Position and orientation in a defined coordinate frame. |
| **Telemetry / telemetrie** | Values displayed on the Driver Station for debugging. |

## Naming rules

- Java classes: `UpperCamelCase` (e.g. `VisionSubsystem`, `TargetDetection`).
- Methods/variables: `lowerCamelCase` (`findLargestColorTarget`, `centerXPx`).
- Constants: `UPPER_SNAKE_CASE` (`FRAME_WIDTH_PX`).
- Units as suffixes: `Px`, `Mm`, `In`, `Deg`, `Ms`; do not mix units silently.
- Dataset classes: `lower_snake_case` (`pollen`, `nectar`, `yellow_game_piece`).
- One meaning per name: `score` should only mean a model's score; use `areaPx`
  for a blob's area rather than calling it "confidence".

## Normalized horizontal offset

`horizontalError = 2 * centerXPx / frameWidthPx - 1`

- `-1`: leftmost edge of raw camera image
- `0`: image center
- `+1`: rightmost edge

This is **not** a distance to the target and is **not** directly a motor power.
Validate camera mounting, mirroring, and rotation before using it for steering.
