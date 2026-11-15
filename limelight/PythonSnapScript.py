# =====================================================================
#  Limelight 3A Python SnapScript
#  - Detects AprilTags (36h11) with OpenCV
#  - Groups them into the 4 field clusters (IDs 30-45)
#  - Picks the cluster CLOSEST TO IMAGE CENTER and returns its tx / ty
#  - Draws a fighter-jet style HUD on the stream
#
#  llpython OUTPUT (read in Java with result.getPythonOutput()):
#    [0] valid        1 = cluster found, 0 = nothing
#    [1] tx           deg to the CLUSTER CENTER, + = right
#    [2] ty           deg to the CLUSTER CENTER, + = up
#    (center is reconstructed from the known tag layout, so it is
#     correct even if only one tag of the cluster is visible)
#    [3] cluster      0 RS, 1 RA, 2 BS, 3 BA, -1 none
#    [4] tagCount     tags in the selected cluster
#    [5] range        meters to cluster center (solvePnP)
#    [6] totalTags    all valid tags seen this frame
#    [7] frame        heartbeat counter
#
#  llrobot INPUT (sent from Java with limelight.updatePythonInputs()):
#    [0] heading deg   [1] auto-aim active (0/1)
#    [2] launcher vel  [3] launcher target vel
#    [4] yaw rate deg/s
#    [5] aim locked (0/1)  [6] robot connected (1)
# =====================================================================

import cv2
import numpy as np
import math
import time

# ---------------- CONFIG ----------------
# ---- Cluster geometry (FTC manual Fig 9-15), inches ----
TAG_SIZE_IN = 3.25      # black square edge
# Horizontal offset of each tag's centerline from the CLUSTER CENTER,
# in order lowest ID -> highest ID (e.g. 30, 31, 32, 33). + = right.
TAG_OFFSETS_IN = [-6.5, -2.75, 2.75, 6.5]
# Clusters whose IDs run right-to-left (index into CLUSTERS).
# This is only the STARTING GUESS: whenever 2+ tags of a cluster are
# visible the script tests both directions and learns the right one,
# then uses it when only 1 tag is visible.
REVERSED_CLUSTERS = {0, 1, 2, 3}
AUTO_ORIENT = True
ORIENT_MAX_ERR_PX = 6.0   # best fit must be this good to learn from
FIT_MAX_ERR_PX = 10.0     # worse than this -> fall back to per-tag centers

# Which cluster to aim at when more than one is visible:
#   "center"  = the one closest to the center of the camera view (default)
#   "highest" = the one highest in the camera image
#   "nearest" = the closest one by distance
SELECT_MODE = "center"
# Keep the current target unless another is this many degrees closer to
# center (stops the aim flip-flopping between two similar clusters)
SWITCH_MARGIN_DEG = 2.0
# All 4 tag centers sit on the cluster centerline (vertical offset 0).
# Reference holes are 2.75 in below that line (used only for drawing).
PANEL_HALF_W_IN = 8.5   # drawn target box half-width
PANEL_HALF_H_IN = 2.5   # drawn target box half-height
IN_TO_M = 0.0254
HFOV_DEG = 54.5         # Limelight 3A horizontal FOV
VFOV_DEG = 42.0         # Limelight 3A vertical FOV
# Standalone lock check (used only when the robot is NOT sending inputs;
# when it is, Java decides "locked" and sends it in llrobot[5])
LOCK_TX_DEG = 0.7
LOCK_TY_TARGET_DEG = 2.15
LOCK_TY_DEG = 1.5
LAUNCHER_READY_TOL = 40 # ticks/s from target counts as "at speed"
DETECT_SCALE = 1.0      # <1.0 detects on a shrunk image (faster, shorter range)

CLUSTERS = [
    ("RED SCORING",   30, 33),
    ("RED AUDIENCE",  34, 37),
    ("BLUE SCORING",  38, 41),
    ("BLUE AUDIENCE", 42, 45),
]

HUD = (0, 255, 0)
HUD_DIM = (0, 130, 0)
WARN = (0, 0, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX
AA = cv2.LINE_AA

_detector = None
_frame = 0
_orient = {}          # cluster -> True if reversed (learned)
_last_target = -1
_last_t = None
_fps = 0.0


# ---------------- DETECTION ----------------
def _make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "ArucoDetector"):          # OpenCV >= 4.7
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return ("new", cv2.aruco.ArucoDetector(dictionary, params))
    params = cv2.aruco.DetectorParameters_create()   # older OpenCV
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return ("old", (dictionary, params))


def _detect(gray):
    kind, det = _detector
    if kind == "new":
        corners, ids, _ = det.detectMarkers(gray)
    else:
        d, p = det
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
    return corners, ids


def _cluster_of(tag_id):
    for i, (_, lo, hi) in enumerate(CLUSTERS):
        if lo <= tag_id <= hi:
            return i
    return -1


def _tag_plane_points(tag_id, rev):
    """Tag corners in the cluster plane (inches, x right, y down),
    in OpenCV's corner order TL, TR, BR, BL."""
    lo = CLUSTERS[_cluster_of(tag_id)][1]
    cx = TAG_OFFSETS_IN[tag_id - lo] * (-1.0 if rev else 1.0)
    h = TAG_SIZE_IN / 2.0
    return np.array([[cx - h, -h], [cx + h, -h], [cx + h, h], [cx - h, h]], np.float64)


def _fit(ts, rev):
    """Homography from the cluster plane to the image. Unique (no pose
    ambiguity) even with one tag. Returns (mean reprojection err px, H)."""
    obj = np.vstack([_tag_plane_points(t["id"], rev) for t in ts])
    img = np.vstack([t["pts"] for t in ts]).astype(np.float64)
    if len(ts) == 1:
        Hm = cv2.getPerspectiveTransform(obj.astype(np.float32), img.astype(np.float32))
    else:
        Hm, _ = cv2.findHomography(obj, img, 0)
    if Hm is None:
        return None
    proj = cv2.perspectiveTransform(obj.reshape(-1, 1, 2), Hm).reshape(-1, 2)
    return float(np.mean(np.linalg.norm(proj - img, axis=1))), Hm


def _apply(Hm, pts):
    return cv2.perspectiveTransform(np.asarray(pts, np.float64).reshape(-1, 1, 2), Hm).reshape(-1, 2)


def _solve_cluster(cl, ts, K, fx):
    default_rev = _orient.get(cl, cl in REVERSED_CLUSTERS)

    # ---- choose layout direction ----
    rev = default_rev
    fit = None
    if len(ts) >= 2 and AUTO_ORIENT:
        f_fwd = _fit(ts, False)
        f_rev = _fit(ts, True)
        cands = [(f, r) for f, r in ((f_fwd, False), (f_rev, True)) if f is not None]
        if cands:
            fit, rev = min(cands, key=lambda c: c[0][0])
            other = [c for c in cands if c[1] != rev]
            clear_winner = not other or other[0][0][0] > 2.0 * fit[0] + 1.0
            if fit[0] < ORIENT_MAX_ERR_PX and clear_winner:
                _orient[cl] = rev
    if fit is None:
        fit = _fit(ts, rev)
    if fit is None:
        return None
    err, Hm = fit

    hw, hh = PANEL_HALF_W_IN, PANEL_HALF_H_IN
    panel_obj = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]]

    if err <= FIT_MAX_ERR_PX:
        center = _apply(Hm, [[0, 0]])[0]
        panel = _apply(Hm, panel_obj)
    else:
        # tags disagree (bad detection) -> each tag votes for the center
        centers = []
        for t in ts:
            f = _fit([t], rev)
            if f is not None:
                centers.append(_apply(f[0], [[0, 0]])[0])
        if not centers:
            return None
        center = np.median(np.array(centers), axis=0)
        panel = _apply(_fit([ts[0]], rev)[1], panel_obj)
        panel += center - _apply(_fit([ts[0]], rev)[1], [[0, 0]])[0]

    # ---- range: solvePnP with the chosen layout, size-based fallback ----
    obj3 = np.hstack([np.vstack([_tag_plane_points(t["id"], rev) for t in ts]),
                      np.zeros((4 * len(ts), 1))]) * IN_TO_M
    img = np.vstack([t["pts"] for t in ts]).astype(np.float64)
    dist = None
    try:
        ok, rvec, tvec = cv2.solvePnP(obj3, img, K, None, flags=cv2.SOLVEPNP_IPPE)
        if ok and tvec[2, 0] > 0:
            dist = float(np.linalg.norm(tvec))
    except cv2.error:
        pass
    if dist is None:
        side = np.mean([np.mean(np.linalg.norm(t["pts"] - np.roll(t["pts"], 1, axis=0), axis=1)) for t in ts])
        dist = TAG_SIZE_IN * IN_TO_M * fx / max(side, 1e-6)

    return center, panel, dist, err, rev


# ---------------- HUD PIECES ----------------
def _text(img, s, x, y, color=HUD, scale=0.4, center=False, thick=1):
    if center:
        (tw, _), _ = cv2.getTextSize(s, FONT, scale, thick)
        x = int(x - tw / 2)
    cv2.putText(img, s, (int(x), int(y)), FONT, scale, color, thick, AA)


def _dashed(img, p1, p2, color, dash=6):
    x1, y1 = p1
    x2, y2 = p2
    length = math.hypot(x2 - x1, y2 - y1)
    n = max(1, int(length / dash))
    for k in range(0, n, 2):
        a = k / n
        b = min(1.0, (k + 1) / n)
        cv2.line(img,
                 (int(x1 + (x2 - x1) * a), int(y1 + (y2 - y1) * a)),
                 (int(x1 + (x2 - x1) * b), int(y1 + (y2 - y1) * b)),
                 color, 1, AA)


def _waterline(img, cx, cy):
    pts = [(cx - 32, cy), (cx - 14, cy), (cx - 7, cy + 8), (cx, cy),
           (cx + 7, cy + 8), (cx + 14, cy), (cx + 32, cy)]
    cv2.polylines(img, [np.array(pts, np.int32)], False, HUD, 2, AA)


def _pitch_ladder(img, cx, cy, fy):
    for deg in (-15, -10, -5, 5, 10, 15):
        y = int(cy - fy * math.tan(math.radians(deg)))
        gap, ln = 45, 45
        tick = 6 if deg > 0 else -6
        for side in (-1, 1):
            xa = cx + side * gap
            xb = cx + side * (gap + ln)
            if deg > 0:
                cv2.line(img, (xa, y), (xb, y), HUD_DIM, 1, AA)
            else:
                _dashed(img, (xa, y), (xb, y), HUD_DIM)
            cv2.line(img, (xb, y), (xb, y + tick), HUD_DIM, 1, AA)
        _text(img, "%d" % abs(deg), cx + gap + ln + 5, y + 4, HUD_DIM, 0.35)
        _text(img, "%d" % abs(deg), cx - gap - ln - 20, y + 4, HUD_DIM, 0.35)


def _heading_tape(img, heading_ftc, w):
    compass = (-heading_ftc) % 360.0      # FTC heading is CCW+, compass is CW+
    cx, y = w // 2, 30
    half_px = int(w * 0.25)
    ppd = half_px / 30.0
    cv2.line(img, (cx - half_px, y), (cx + half_px, y), HUD, 1, AA)
    d = int(math.floor((compass - 30) / 5.0) * 5)
    while d <= compass + 30:
        x = int(cx + (d - compass) * ppd)
        if abs(x - cx) <= half_px:
            major = d % 10 == 0
            cv2.line(img, (x, y), (x, y - (9 if major else 4)), HUD, 1, AA)
            if major:
                _text(img, "%02d" % ((d % 360) // 10), x, y - 13, HUD, 0.38, True)
        d += 5
    tri = np.array([(cx, y + 2), (cx - 5, y + 10), (cx + 5, y + 10)], np.int32)
    cv2.polylines(img, [tri], True, HUD, 1, AA)
    cv2.rectangle(img, (cx - 18, y + 12), (cx + 18, y + 28), HUD, 1, AA)
    _text(img, "%03d" % int(round(compass)) if compass < 359.5 else "000",
          cx, y + 25, HUD, 0.42, True)


def _vtape(img, x, h, value, step, ppu, label, left, fmt="%d"):
    top, bot, cy = int(h * 0.28), int(h * 0.72), h // 2
    inward = 1 if left else -1
    cv2.line(img, (x, top), (x, bot), HUD, 1, AA)
    span = (bot - top) / 2.0 / ppu
    v = math.floor((value - span) / step) * step
    i = 0
    while v <= value + span and i < 200:
        y = int(cy - (v - value) * ppu)
        if top <= y <= bot:
            major = int(round(v / step)) % 2 == 0
            cv2.line(img, (x, y), (x + inward * (9 if major else 4), y), HUD, 1, AA)
            if major:
                tx = x - 44 if left else x + 6
                _text(img, fmt % v, tx, y + 4, HUD_DIM, 0.33)
        v += step
        i += 1
    # readout box
    if left:
        cv2.rectangle(img, (x - 56, cy - 10), (x - 4, cy + 10), (0, 0, 0), -1)
        cv2.rectangle(img, (x - 56, cy - 10), (x - 4, cy + 10), HUD, 1, AA)
        _text(img, fmt % value, x - 30, cy + 5, HUD, 0.42, True)
    else:
        cv2.rectangle(img, (x + 4, cy - 10), (x + 56, cy + 10), (0, 0, 0), -1)
        cv2.rectangle(img, (x + 4, cy - 10), (x + 56, cy + 10), HUD, 1, AA)
        _text(img, fmt % value, x + 30, cy + 5, HUD, 0.42, True)
    _text(img, label, x, top - 8, HUD, 0.38, True)


def _flight_path_marker(img, cx, cy, yaw_rate):
    x = int(cx + max(-80, min(80, -yaw_rate * 0.8)))
    y = cy + 22
    cv2.circle(img, (x, y), 6, HUD, 1, AA)
    cv2.line(img, (x - 16, y), (x - 6, y), HUD, 1, AA)
    cv2.line(img, (x + 6, y), (x + 16, y), HUD, 1, AA)
    cv2.line(img, (x, y - 6), (x, y - 12), HUD, 1, AA)


def _jet_icon(img, ox, oy, bank_deg, scale=1.4):
    right = [(0, -22), (2, -16), (3, -6), (20, 4), (20, 8), (3, 5),
             (3, 13), (9, 18), (9, 21), (0, 19)]
    full = right + [(-px, py) for px, py in reversed(right[1:-1])]
    a = math.radians(bank_deg)
    ca, sa = math.cos(a), math.sin(a)
    pts = np.array([(ox + scale * (px * ca - py * sa),
                     oy + scale * (px * sa + py * ca)) for px, py in full], np.int32)
    cv2.fillPoly(img, [pts], HUD_DIM, AA)
    cv2.polylines(img, [pts], True, HUD, 1, AA)
    cv2.circle(img, (ox, oy), int(34 * scale / 1.4), HUD_DIM, 1, AA)


def _brackets(img, x1, y1, x2, y2, color, L=10, t=1):
    for (x, y, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1),
                           (x1, y2, 1, -1), (x2, y2, -1, -1)):
        cv2.line(img, (x, y), (x + dx * L, y), color, t, AA)
        cv2.line(img, (x, y), (x, y + dy * L), color, t, AA)


def _draw_hud(img, w, h, fy, tags, summaries, best, rb, blink):
    heading, aim_active, launch_vel, launch_target, yaw_rate = rb[:5]
    cx, cy = w // 2, h // 2

    _pitch_ladder(img, cx, cy, fy)
    _heading_tape(img, heading, w)
    _vtape(img, 70, h, launch_vel, 50, 0.5, "SPD", True)
    rng_in = best["dist"] * 39.37 if best else 0.0
    _vtape(img, w - 70, h, rng_in, 6, 2.5, "RNG", False)
    _waterline(img, cx, cy)
    _flight_path_marker(img, cx, cy, yaw_rate)

    # raw tags
    for t in tags:
        col = HUD_DIM if t["cluster"] >= 0 else (0, 90, 160)
        cv2.polylines(img, [t["pts"].astype(np.int32)], True, col, 1, AA)
        c = t["pts"].mean(axis=0)
        _text(img, str(t["id"]), c[0], c[1] + 4, col, 0.35, True)

    # secondary clusters
    for s in summaries:
        if best is not None and s["cluster"] == best["cluster"]:
            continue
        x, y, bw, bh = cv2.boundingRect(s["panel"].astype(np.int32))
        _brackets(img, x, y, x + bw, y + bh, HUD_DIM, 6)
        _text(img, "%s %s" % (CLUSTERS[s["cluster"]][0], "R" if s["rev"] else "F"),
              x, y - 6, HUD_DIM, 0.33)

    if rb[6] > 0.5:                       # robot is sending its own lock state
        locked = best is not None and rb[5] > 0.5
    else:                                  # standalone (web UI) fallback
        locked = (best is not None and abs(best["tx"]) < LOCK_TX_DEG
                  and abs(best["ty"] - LOCK_TY_TARGET_DEG) < LOCK_TY_DEG)

    if best is not None:
        panel = best["panel"].astype(np.int32)
        tcx, tcy = int(best["center"][0]), int(best["center"][1])
        x1, y1 = panel[:, 0].min(), panel[:, 1].min()
        x2, y2 = panel[:, 0].max(), panel[:, 1].max()

        if locked:
            cv2.polylines(img, [panel], True, HUD, 2, AA)
            cv2.line(img, tuple(panel[0]), tuple(panel[2]), HUD, 1, AA)
            cv2.line(img, tuple(panel[1]), tuple(panel[3]), HUD, 1, AA)
        else:
            _brackets(img, x1, y1, x2, y2, HUD, 12, 2)
            _dashed(img, (cx, cy), (tcx, tcy), HUD_DIM, 8)
            _text(img, "TRK", (x1 + x2) // 2, y1 - 10, HUD, 0.45, True)

        # diamond pipper on the reconstructed cluster center
        d = 7
        dia = np.array([(tcx, tcy - d), (tcx + d, tcy), (tcx, tcy + d), (tcx - d, tcy)], np.int32)
        cv2.polylines(img, [dia], True, HUD, 1, AA)
        cv2.circle(img, (tcx, tcy), 1, HUD, -1, AA)

        _text(img, "%s  %s  ERR %.1f" % (CLUSTERS[best["cluster"]][0],
                                         "REV" if best["rev"] else "FWD", best["err"]),
              x1, y2 + 16, HUD, 0.4)

    # TARGET LOCKED banner (bottom center)
    if locked:
        msg = "TARGET LOCKED"
        (tw, th), _ = cv2.getTextSize(msg, FONT, 0.7, 2)
        bx1, by1 = cx - tw // 2 - 10, h - 42
        bx2, by2 = cx + tw // 2 + 10, h - 14
        cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 0, 0), -1)
        cv2.rectangle(img, (bx1, by1), (bx2, by2), HUD, 2, AA)
        if blink:
            cv2.rectangle(img, (bx1 + 3, by1 + 3), (bx2 - 3, by2 - 3), HUD_DIM, 1, AA)
        _text(img, msg, cx, by2 - 8, HUD, 0.7, True, 2)

    # launcher ready / shoot cue
    at_speed = launch_target > 0 and abs(launch_vel - launch_target) < LAUNCHER_READY_TOL
    if locked and at_speed:
        _text(img, "SHOOT", cx, cy + 90, HUD, 0.8, True, 2)
    elif aim_active > 0.5 and best is None and blink:
        _text(img, "SEARCH", cx, cy + 90, WARN, 0.6, True, 2)

    # data block bottom-left
    yb = h - 62
    if best is not None:
        _text(img, "TGT %s" % CLUSTERS[best["cluster"]][0], 10, yb)
        _text(img, "TX %+5.1f  TY %+5.1f" % (best["tx"], best["ty"]), 10, yb + 15)
        _text(img, "RNG %5.1f IN  TAGS %d" % (rng_in, best["count"]), 10, yb + 30)
    else:
        _text(img, "TGT ----", 10, yb, HUD_DIM)
    _text(img, "%4.1f FPS" % _fps, 10, yb + 45, HUD_DIM, 0.35)

    # mode block bottom-right + attitude jet
    _text(img, "AUTO AIM" if aim_active > 0.5 else "MAN", w - 90, h - 15,
          HUD if aim_active > 0.5 else HUD_DIM, 0.45)
    _text(img, "ARM" if at_speed else "SAFE", w - 90, h - 32,
          HUD if at_speed else HUD_DIM, 0.45)
    _jet_icon(img, w - 135, h - 40, max(-45, min(45, -yaw_rate * 0.3)), 1.0)


# ---------------- MAIN ENTRY ----------------
def runPipeline(image, llrobot):
    global _detector, _frame, _last_t, _fps, _last_target
    _frame += 1
    if _detector is None:
        _detector = _make_detector()

    now = time.time()
    if _last_t is not None and now > _last_t:
        _fps = 0.9 * _fps + 0.1 * (1.0 / (now - _last_t))
    _last_t = now

    h, w = image.shape[:2]
    fx = (w / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    fy = (h / 2.0) / math.tan(math.radians(VFOV_DEG / 2.0))
    cx0, cy0 = w / 2.0, h / 2.0

    rb = [float(v) for v in llrobot] if llrobot is not None else []
    rb += [0.0] * (8 - len(rb))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if DETECT_SCALE != 1.0:
        gray = cv2.resize(gray, None, fx=DETECT_SCALE, fy=DETECT_SCALE,
                          interpolation=cv2.INTER_AREA)
    corners, ids = _detect(gray)

    K = np.array([[fx, 0, cx0], [0, fy, cy0], [0, 0, 1]], np.float64)

    # ---- per-tag detections ----
    tags = []
    if ids is not None:
        for c, tid in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2).astype(np.float64) / DETECT_SCALE
            tags.append({"id": int(tid), "cluster": _cluster_of(int(tid)), "pts": pts})

    # ---- group into clusters, reconstruct each cluster center ----
    groups = {}
    for t in tags:
        if t["cluster"] >= 0:
            groups.setdefault(t["cluster"], []).append(t)

    summaries = []
    for cl, ts in groups.items():
        solved = _solve_cluster(cl, ts, K, fx)
        if solved is None:
            continue
        center_px, panel_px, dist, fit_err, rev = solved
        summaries.append({
            "cluster": cl,
            "count": len(ts),
            "center": center_px,
            "panel": panel_px,
            "tx": math.degrees(math.atan((center_px[0] - cx0) / fx)),
            "ty": math.degrees(math.atan((cy0 - center_px[1]) / fy)),
            "dist": dist,
            "err": fit_err,
            "rev": rev,
            "pts": np.vstack([t["pts"] for t in ts]),
        })

    if not summaries:
        best = None
    elif SELECT_MODE == "center":
        def off_center(sm):
            d = math.hypot(sm["tx"], sm["ty"])
            return d - SWITCH_MARGIN_DEG if sm["cluster"] == _last_target else d
        best = min(summaries, key=lambda sm: (off_center(sm), -sm["count"]))
    elif SELECT_MODE == "nearest":
        best = min(summaries, key=lambda s: (s["dist"], -s["count"]))
    else:
        # highest on screen = largest ty; ties go to more tags
        best = max(summaries, key=lambda s: (s["ty"], s["count"]))

    _last_target = best["cluster"] if best is not None else -1

    blink = (_frame // 6) % 2 == 0
    _draw_hud(image, w, h, fy, tags, summaries, best, rb, blink)

    valid_tags = sum(1 for t in tags if t["cluster"] >= 0)
    if best is not None:
        contour = best["panel"].astype(np.int32).reshape(-1, 1, 2)
        llpython = [1.0, best["tx"], best["ty"], float(best["cluster"]),
                    float(best["count"]), best["dist"], float(valid_tags),
                    float(_frame % 100000)]
    else:
        contour = np.array([[]])
        llpython = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, float(valid_tags),
                    float(_frame % 100000)]

    return contour, image, llpython
