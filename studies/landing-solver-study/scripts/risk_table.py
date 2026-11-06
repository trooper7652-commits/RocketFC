"""Prototype of candidate C: risk-aware ignition table (analysis tool, numpy).

For a descent speed v (as the solver will see it) the fire command is issued at
altitude H. Over an uncertainty distribution (delay, mass, CdA, thrust scale,
estimator residual error) we simulate the 1-D vertical flight INCLUDING what
happens after a hop (stop in the air -> climb while thrust > weight -> burnout
-> fall), and record the speed at first ground contact. Because altitude never
enters the dynamics, one integration per draw gives the touchdown speed for
EVERY candidate H at once: contact happens the first time the downward
displacement reaches H.

choose H*(v) = argmin_H  mean_j  vtd_j(H)            (expected touchdown speed)
"""
import math

import numpy as np

G = 9.80665
RHO = 1.225


class Curve:
    def __init__(self, path, scale=1.0):
        pts = [(0.0, 0.0)]
        header = None
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith(";"):
                    continue
                if header is None:
                    p = s.split()
                    header = p
                    self.prop = float(p[4])
                    continue
                t, F = s.split()[:2]
                pts.append((float(t), float(F) * scale))
        pts.sort()
        self.t = np.array([p[0] for p in pts])
        self.f = np.array([p[1] for p in pts])
        self.burn = float(self.t[-1])
        # dense grid for fast vectorized lookup
        self.dt = 0.0005
        self.grid = np.arange(0.0, self.burn + self.dt, self.dt)
        self.F = np.interp(self.grid, self.t, self.f, left=0.0, right=0.0)
        I = np.concatenate([[0.0], np.cumsum(0.5 * (self.F[1:] + self.F[:-1]) * self.dt)])
        self.I = I
        self.Itot = float(I[-1])

    def thrust(self, tau):
        """vectorized: thrust at time since ignition tau (array)."""
        out = np.interp(tau, self.grid, self.F, left=0.0, right=0.0)
        return out

    def burned_frac(self, tau):
        return np.interp(tau, self.grid, self.I, left=0.0, right=self.Itot) / self.Itot


def simulate(curve, v0, delay, mass, cda, tscale, dt=0.002, tmax=14.0,
             theta_on=None, tau_r=0.25):
    """Vectorized RK4 of x (displacement, +up, x0=0) and v (+up) for J draws.
    Optional tilt: thrust's vertical share is cos(theta(t)) with
    theta(t) = theta_on * exp(-(t - delay)/tau_r) after thrust onset (the TVC
    righting the vehicle). Returns float32 arrays X[n, J], V[n, J]."""
    J = len(v0)
    x = np.zeros(J)
    v = -np.abs(v0).astype(float)
    k = 0.5 * RHO * cda
    n = int(tmax / dt) + 1
    X = np.empty((n, J), dtype=np.float32)
    V = np.empty((n, J), dtype=np.float32)

    def acc(t, vv):
        tau = t - delay
        m = mass - curve.prop * curve.burned_frac(tau)
        T = tscale * curve.thrust(tau)
        if theta_on is not None:
            th = theta_on * np.exp(-np.maximum(tau, 0.0) / tau_r)
            T = T * np.cos(th)
        return (T - k * vv * np.abs(vv)) / m - G

    t = 0.0
    for i in range(n):
        X[i] = x
        V[i] = v
        a1 = acc(t, v)
        v2 = v + 0.5 * dt * a1
        a2 = acc(t + 0.5 * dt, v2)
        v3 = v + 0.5 * dt * a2
        a3 = acc(t + 0.5 * dt, v3)
        v4 = v + dt * a3
        a4 = acc(t + dt, v4)
        x = x + dt * (v + 2 * v2 + 2 * v3 + v4) / 6.0
        v = v + dt * (a1 + 2 * a2 + 2 * a3 + a4) / 6.0
        t += dt
    return X, V


def contact_speed_curves(X, V, levels):
    """For each draw j and each level L (m, > 0), speed at the first time the
    displacement reaches -L. Returns array (J, len(levels)); nan if never."""
    n, J = X.shape
    out = np.full((J, len(levels)), np.nan)
    for j in range(J):
        x = X[:, j].astype(float)
        runmin = np.minimum.accumulate(x)
        depth = -runmin  # nondecreasing
        idx = np.searchsorted(depth, levels, side="left")  # first i with depth>=L
        ok = (idx > 0) & (idx < n)
        i1 = idx[ok]
        i0 = i1 - 1
        d0, d1 = depth[i0], depth[i1]
        f = np.where(d1 > d0, (levels[ok] - d0) / np.maximum(d1 - d0, 1e-12), 1.0)
        vv = V[i0, j] + f * (V[i1, j] - V[i0, j])
        out[j, ok] = np.abs(vv)
    return out


def best_altitude(curve, v_est, draws, levels, objective="mean"):
    """draws: dict of arrays (delay, mass, cda, tscale, eh, ev), all length J.
    Estimated state at fire: altitude H (decision), speed v_est. True state:
    h = H - eh, v_true = v_est - ev (ev >0 = KF reads too slow; v down-positive
    here, so true down-speed = v_est + ev_down). Returns (H*, stats dict)."""
    vtrue = v_est + draws["ev_down"]
    X, V = simulate(curve, vtrue, draws["delay"], draws["mass"], draws["cda"],
                    draws["tscale"])
    C = contact_speed_curves(X, V, levels)          # (J, L) vs TRUE contact level
    # decision H maps to true level H - eh (per draw): shift by interpolation
    Hs = levels
    J = C.shape[0]
    M = np.empty((J, len(Hs)))
    for j in range(J):
        M[j] = np.interp(Hs - draws["eh"][j], levels, C[j], left=np.nan, right=np.nan)
    if objective == "mean":
        score = np.nanmean(M, axis=0)
    elif objective == "median":
        score = np.nanmedian(M, axis=0)
    elif objective == "rms":
        score = np.sqrt(np.nanmean(M * M, axis=0))
    else:
        raise ValueError(objective)
    k = int(np.nanargmin(score))
    col = M[:, k]
    return float(Hs[k]), {
        "score": float(score[k]), "median": float(np.nanmedian(col)),
        "p90": float(np.nanpercentile(col, 90)), "mean": float(np.nanmean(col)),
        "score_curve": score,
    }


def nominal_altitude(curve, v, mass, cda, delay, v_aim=0.0):
    """Exact zero-uncertainty stop altitude (first time down-speed <= v_aim)."""
    X, V = simulate(curve, np.array([v]), np.array([delay]), np.array([mass]),
                    np.array([cda]), np.array([1.0]), dt=0.0005, tmax=8.0)
    x, vv = X[:, 0].astype(float), V[:, 0].astype(float)
    i = int(np.argmax(-vv <= v_aim))
    if -vv[i] > v_aim:
        return float("nan")
    f = (-vv[i - 1] - v_aim) / max(vv[i] - vv[i - 1], 1e-12)
    return float(-(x[i - 1] + f * (x[i] - x[i - 1])))
