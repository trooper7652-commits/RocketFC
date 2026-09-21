# VENDORED from SensorServoBenchTest/viz/quatmath.py on 2026-09-21.
#
# Copied rather than imported across projects so RocketFC stays self-contained
# and clonable on its own (the same reasoning, in the opposite direction, as
# SensorServoBenchTest/sync_core.py vendoring the flight headers into there).
#
# Unlike sync_core.py's copies this is NOT auto-synced and IS edited here:
# the bench rig's vehicle never translates, so the flight viewer needs a
# moving origin and a metres-scaled ground grid. Fixes that belong to both
# should be made in both; the two lineages are expected to drift.
"""
The subset of RocketFC's quaternion math that the host needs.

Deliberately a direct port of src/core/quat.h rather than a general-purpose
rotation library, so the host and the flight computer agree by construction:

  - Hamilton convention, stored w-first as (w, x, y, z)
  - a quaternion represents the BODY -> WORLD rotation, i.e.
    v_world = rotate(q, v_body)
  - world +Z is up; body +Z points out the nose

If you ever find yourself reaching for scipy's Rotation here, check its
convention first -- it stores (x, y, z, w).
"""

import numpy as np

WORLD_UP = np.array([0.0, 0.0, 1.0])
IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


def normalize(v):
    """Unit vector, or an arbitrary safe direction for a degenerate input."""
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    return np.asarray(v, dtype=float) / n


def quat_normalize(q):
    n = float(np.linalg.norm(q))
    if n < 1e-9:
        return IDENTITY.copy()
    return np.asarray(q, dtype=float) / n


def rotate(q, v):
    """Rotate a body-frame vector into the world frame (Quat::rotate)."""
    w, x, y, z = q
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return np.asarray(v, dtype=float) + w * t + np.cross(qv, t)


def rotate_inv(q, v):
    """Rotate a world-frame vector into the body frame (Quat::rotateInv)."""
    w, x, y, z = q
    return rotate(np.array([w, -x, -y, -z]), v)


def from_two_vectors(f, t):
    """
    Shortest rotation taking direction `f` to direction `t`.

    Port of Quat::fromTwoVectors (quat.h). This is exactly the operation
    Ahrs::initFromAccel performs, which is why the accel-only "ghost" attitude
    in the visualizer is computed with it -- same math, same conventions, no
    gyro involved.
    """
    f = normalize(f)
    t = normalize(t)
    d = float(np.dot(f, t))
    if d < -0.999999:
        # Antiparallel: any perpendicular axis, rotated 180 degrees.
        axis = np.cross(np.array([1.0, 0.0, 0.0]), f)
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(np.array([0.0, 1.0, 0.0]), f)
        axis = normalize(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])
    c = np.cross(f, t)
    return quat_normalize(np.array([1.0 + d, c[0], c[1], c[2]]))


def gl_matrix(q):
    """
    16-element float32 body->world rotation, laid out for glMultMatrixf.

    OpenGL consumes matrices in COLUMN-major order, while numpy is row-major,
    so the rotation matrix is transposed on the way out. Getting this backwards
    produces a model that rotates the right amount in the wrong direction --
    subtle, and exactly the class of bug this tool exists to expose, so it is
    covered by a self-test in this module.
    """
    w, x, y, z = quat_normalize(q)
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    m = np.eye(4)
    m[:3, :3] = r
    return np.ascontiguousarray(m.T, dtype=np.float32).ravel()


def tilt_deg(q):
    """Angle between the body long axis (+Z) and world up (Ahrs::tiltRad)."""
    zb = rotate(q, WORLD_UP)
    return float(np.degrees(np.arccos(np.clip(zb[2], -1.0, 1.0))))


def angle_between_deg(qa, qb):
    """Total rotation angle separating two attitudes, in degrees."""
    d = abs(float(np.dot(quat_normalize(qa), quat_normalize(qb))))
    return float(np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0))))


def tilt_angle_between_deg(qa, qb):
    """
    Angle between the two attitudes' long axes only, ignoring heading.

    The accelerometer observes tilt but not heading, so comparing a full
    attitude against an accel-derived one with angle_between_deg() would report
    a large, meaningless disagreement as soon as the AHRS heading drifts. This
    compares only the quantity both actually know.
    """
    za = rotate(qa, WORLD_UP)
    zb = rotate(qb, WORLD_UP)
    return float(np.degrees(np.arccos(np.clip(float(np.dot(za, zb)), -1.0, 1.0))))


def quat_from_accel(accel_body):
    """
    Attitude implied by gravity alone -- the "ghost".

    On a bench the only specific force is gravity, so the accelerometer reads
    +1 g along whatever body axis points up. The rotation carrying that reading
    onto world up is a heading-free attitude: its tilt is meaningful, its yaw
    is arbitrary.
    """
    return from_two_vectors(accel_body, WORLD_UP)


def dominant_body_axis(accel_body):
    """
    Which body axis gravity currently lies along, e.g. ("+Z", 9.79).

    Used by the axis-identification panel: with the vehicle standing nose up,
    the answer must be +Z. Anything else means IMU_R_SB in config.h does not
    match how the BMI088 is actually mounted.
    """
    a = np.asarray(accel_body, dtype=float)
    i = int(np.argmax(np.abs(a)))
    name = "XYZ"[i]
    sign = "+" if a[i] >= 0 else "-"
    return sign + name, float(abs(a[i]))


def _self_test():
    """Run with: python quatmath.py"""
    # A 90 deg rotation about world/body X takes body +Z (nose) to world -Y.
    s = np.sqrt(0.5)
    q = np.array([s, s, 0.0, 0.0])
    nose = rotate(q, WORLD_UP)
    assert np.allclose(nose, [0, -1, 0], atol=1e-6), nose
    assert abs(tilt_deg(q) - 90.0) < 1e-4, tilt_deg(q)

    # rotate_inv undoes rotate.
    v = np.array([0.3, -0.7, 0.2])
    assert np.allclose(rotate_inv(q, rotate(q, v)), v, atol=1e-6)

    # Identity attitude is upright and agrees with a nose-up accel reading.
    assert abs(tilt_deg(IDENTITY)) < 1e-6
    ghost = quat_from_accel(np.array([0.0, 0.0, 9.81]))
    assert abs(tilt_deg(ghost)) < 1e-6
    assert dominant_body_axis(np.array([0.0, 0.0, 9.81]))[0] == "+Z"
    assert dominant_body_axis(np.array([0.0, -9.7, 0.4]))[0] == "-Y"

    # A vehicle leaning so that gravity reads partly along body +X must produce
    # a ghost with matching tilt.
    a = np.array([np.sin(np.radians(20)), 0.0, np.cos(np.radians(20))]) * 9.81
    assert abs(tilt_deg(quat_from_accel(a)) - 20.0) < 1e-3

    # gl_matrix must be the COLUMN-major form of the body->world rotation:
    # reshaping it back in column-major order and applying it to a body vector
    # has to agree with rotate().
    m = gl_matrix(q).reshape(4, 4, order="F")
    got = m[:3, :3] @ v
    assert np.allclose(got, rotate(q, v), atol=1e-6), (got, rotate(q, v))

    # Heading-only differences must not register as tilt disagreement.
    yaw = np.array([np.cos(np.radians(60)), 0, 0, np.sin(np.radians(60))])
    assert tilt_angle_between_deg(IDENTITY, yaw) < 1e-4
    assert angle_between_deg(IDENTITY, yaw) > 100.0

    print("quatmath self-test passed")


if __name__ == "__main__":
    _self_test()
