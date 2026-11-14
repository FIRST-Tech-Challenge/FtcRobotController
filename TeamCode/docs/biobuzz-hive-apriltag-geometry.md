# BIOBUZZ Hive and AprilTag Geometry

*Reference for shooter, turret, and vision code. Source: Competition Manual TU02, Section 9 (Figs 9-8 to 9-11, 9-15 to 9-17) and the FTC SDK 12.0 release notes. Values marked **(derived)** were calculated from the manual's drawings and have **not** been measured on a real field. Treat them as starting points and replace them with measurements.*

## Published dimensions

| Item | Value | Source |
|---|---|---|
| Pivot height above tiles | 43.95 in | Fig 9-8 |
| Frame width (along pivot axis) / depth | 49.46 in / 38.95 in | Fig 9-8 |
| Hive rest angle | 30° from level | Fig 9-10 |
| Raised cell opening, bottom / top | 53.5 in / 65.6 in | Fig 9-10 |
| Lowest point of hive | 30.6 in | Fig 9-10 |
| Red hive ↔ blue hive, center to center | 25.5 in (same frame, side by side) | Fig 9-10 |
| Cell spacing / cell length / overall span | 18.84 / 12.04 / 42.91 in | Fig 9-9 |
| Cell opening (pentagon) | 20 in wide × 14 in tall | Fig 9-11 |
| AprilTag size / family | 3.25 in, 36h11 | 9.9 |
| Tag centers from cluster center | ±2.75 in (inner), ±6.5 in (outer) | Fig 9-15 |

## Tag IDs

| Cell | IDs |
|---|---|
| Red, far (scoring) side | 30, 31, 32, 33 |
| Red, audience side | 34, 35, 36, 37 |
| Blue, audience side | 38, 39, 40, 41 |
| Blue, far (scoring) side | 42, 43, 44, 45 |

```java
// Suggested constants
public static final int[] RED_FAR_CELL_TAGS       = {30, 31, 32, 33};
public static final int[] RED_AUDIENCE_CELL_TAGS  = {34, 35, 36, 37};
public static final int[] BLUE_AUDIENCE_CELL_TAGS = {38, 39, 40, 41};
public static final int[] BLUE_FAR_CELL_TAGS      = {42, 43, 44, 45};
```

## How the cell and tags are oriented

- The cell's **opening is its outer end**. When the cell is raised, the opening is a window facing outward that **leans back 30° from vertical**. Balls that enter roll back to the closed inner end.
- The four tags are on one sticker on the cell's **bottom face**. When the cell is raised, they face **down and 30° outward, toward the robots shooting at that cell**, so the tag face points 60° below horizontal.
- The cell's support arm runs through the gap in the middle of the cluster. From some angles it **hides the inner tags or one pair**. Code must work with whatever tags are visible.
- **After a tip, the shooting side flips.** A raised far-side cell faces the far wall, and a raised audience-side cell faces the audience.

## Derived positions (raised cell)

Measured horizontally from the pivot line, toward the side the raised cell faces.

| Feature | Height | Horizontal from pivot |
|---|---|---|
| Tag cluster center, raised cell (derived) | ~50 in | ~15 in |
| Opening center, the aim point (derived) | ~59.5 in | ~17.5 in |
| Tag cluster center, lowered cell (derived) | ~34 in | ~12.5 in on the far side of the pivot |
| Frame base bars, the closest the robot can get (derived) | floor | ~19.5 in |

## Rules for vision code

1. **Filter by alliance.** The other alliance's hive is 25.5 in away on the same frame, and its tags will be in view.
2. **Both of your clusters can be visible at once**, because the lowered cell's tags face the same direction. **Target the higher one** (larger `ty`, ~50 in versus ~34 in). Whichever ID group is higher tells you which cell is raised.
3. **The aim point is the opening, not the tags.** It's about 9.5 in above the tag cluster center and about 2.7 in closer to the robot, but in line side to side. Turning the turret onto the cluster center aims it at the opening.
4. **FTC SDK 12.0 note:** the SDK's AprilTag library (`getCurrentGameTagLibrary()`) treats each BIOBUZZ sticker as a **cluster** whose **origin is already set at the center of the cell opening**. With `VisionPortal`, the cluster pose gives the aim point directly, even when only one tag is visible. The Limelight doesn't know about clusters, so with the Limelight we apply the offset ourselves.
5. **Don't use these tags for field localization.** The SDK release notes say so directly: *"since BIOBUZZ AprilTags move, they are not suitable for absolute Field Localization."* The hive tips. Use Pinpoint odometry for position and the tags only for aiming.

## Camera math

For a lens at height `h` inches, the tags are `Δh = 50 − h` inches above it (derived).

- Look-up angle at horizontal distance `d`: `θ = atan(Δh / d)`
- Horizontal distance from the Limelight: `d = Δh / tan(cameraTilt + ty)`
- Distance to the opening ≈ `d − 2.7 in`
- Share of the tag's face the camera sees: `cos(60° − θ)`

Example, lens at 10 in:

| Horizontal distance | θ | Tag face visible |
|---|---|---|
| 2 ft | 59° | ~100% |
| 4 ft | 40° | 94% |
| 6 ft | 29° | 86% |
| 8 ft | 23° | 79% |
| 10 ft | 18° | 75% |

The viewing angle is fine. **Tag size limits range.** A 3.25 in tag probably becomes unreliable around 8–10 ft; measure it.

## Lighting

The tags sit in the cell's shadow, and the camera looks up toward the arena lights. Expect to retune exposure and gain on a real field; our usual 1000 / 8 may be too dark.

## Verify on the real field

- [ ] Tag cluster center height (~50 in derived) `[#88]`
- [ ] Opening center height (~59.5 in derived) `[#88]`
- [ ] Maximum reliable Limelight detection range on these tags under gym lighting `[#64]`
