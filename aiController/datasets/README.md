# Dataset conventions

For a trained detector (e.g. YOLO), first agree on **exact object classes**.
Example labels are `pollen` and `nectar`, which are placeholders until verified
against the team's field elements and robot-camera viewpoint.

Keep collected images separated by capture session / camera / lighting, and
split sessions into **training, validation and testing** without putting
near-identical video frames into different splits.

An example YOLO directory layout (create it when you have labeled images):

```text
datasets/
  dataset.yaml               # copy/edit dataset.yaml.example
  images/
    train/                    # images: .jpg or .png
    val/
    test/
  labels/
    train/                    # one .txt per image
    val/
    test/
```

Each YOLO label line is:
`class_id center_x center_y box_width box_height`
with the last four values **normalized to [0, 1]**. Do not use absolute
pixel coordinates in YOLO label files.

For example, `0 0.5 0.5 0.2 0.3` means class 0 in a box centered in the image.
Class IDs start at 0 and must match the `names` in `dataset.yaml` exactly.

Workflow: collect -> annotate -> inspect labels -> split sessions -> train on
PC -> evaluate false positives/negatives -> test on the real field.

**Do not include photos of people's faces or sensitive information** in
competition training material. Keep raw recordings outside Git if they are
large or contain personal data; consider a separate approved storage location.
