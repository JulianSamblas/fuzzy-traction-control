"""
Predictive fuzzy traction control for racing motorcycles (Mamdani inference).

Inputs : rear-wheel slip (0-20 %) and lean angle (0-60 deg)
Output : engine torque cut (0-100 %)

The membership functions and the 5-rule base are the same as in the
FuzzyCLIPS implementation (clips/BC_MotoGP.clp), so both give the same result.

Usage:
    python fuzzy_traction_control.py                 # case study: slip 12 %, lean 40 deg
    python fuzzy_traction_control.py --slip 8 --lean 50
    python fuzzy_traction_control.py --plots         # also save the figures in images/

Author: Julián Samblás Caballero - MSc Artificial Intelligence (VIU), Approximate Reasoning
"""

import argparse
import os

import numpy as np

# ---------------------------------------------------------------------------
# Linguistic variables: each fuzzy set is defined by its vertices (x, degree),
# exactly like a FuzzyCLIPS deftemplate. Values between vertices are linear.
# ---------------------------------------------------------------------------
SLIP = {                         # rear-wheel slip, 0-20 %
    "low":    ([0, 4, 10],   [1, 1, 0]),
    "medium": ([4, 10, 16],  [0, 1, 0]),
    "high":   ([10, 16, 20], [0, 1, 1]),
}
LEAN = {                         # lean angle, 0-60 degrees
    "low":    ([0, 15, 30],  [1, 1, 0]),
    "medium": ([15, 30, 45], [0, 1, 0]),
    "high":   ([30, 45, 60], [0, 1, 1]),
}
TORQUE_CUT = {                   # torque cut applied by the ECU, 0-100 %
    "none":   ([0, 10, 30],   [1, 1, 0]),
    "soft":   ([10, 30, 50],  [0, 1, 0]),
    "medium": ([30, 50, 70],  [0, 1, 0]),
    "strong": ([50, 70, 100], [0, 1, 1]),
}

# ---------------------------------------------------------------------------
# Rule base: (slip label, lean label or None, torque-cut label)
# None means the rule does not look at the lean angle.
# ---------------------------------------------------------------------------
RULES = [
    ("high",   "high",   "strong"),  # R1: slipping with the bike leaned over -> highside risk
    ("high",   "low",    "medium"),  # R2: slipping with the bike upright -> controllable drift
    ("medium", "high",   "medium"),  # R3: moderate slip, little contact patch available
    ("medium", "medium", "soft"),    # R4: moderate slip at medium lean -> fine correction
    ("low",    None,     "none"),    # R5: no slip -> do not intervene
]

OUTPUT_UNIVERSE = np.linspace(0, 100, 2001)


def membership(fuzzy_set, x):
    """Membership degree of x (scalar or array): linear interpolation between vertices."""
    xs, ys = fuzzy_set
    return np.interp(x, xs, ys)


def infer(slip, lean):
    """
    Mamdani inference for one crisp measurement.

    Returns a dict with:
      - fuzzified : membership degree of the inputs in every label
      - alphas    : firing strength of each rule (min t-norm)
      - aggregated: output fuzzy set (max of the clipped consequents)
      - cog, mom  : defuzzified torque cut (centre of gravity, mean of maximum)
    """
    # 1) Fuzzification
    fuzzified = {"slip": {}, "lean": {}}
    for label in SLIP:
        fuzzified["slip"][label] = float(membership(SLIP[label], slip))
    for label in LEAN:
        fuzzified["lean"][label] = float(membership(LEAN[label], lean))

    # 2) Rule evaluation (AND = minimum) and 3) aggregation (OR = maximum)
    aggregated = np.zeros_like(OUTPUT_UNIVERSE)
    alphas = []
    for i, (slip_label, lean_label, out_label) in enumerate(RULES, start=1):
        alpha = fuzzified["slip"][slip_label]
        if lean_label is not None:
            alpha = min(alpha, fuzzified["lean"][lean_label])
        alphas.append((f"R{i}", out_label, alpha))

        if alpha > 0:
            clipped = np.minimum(membership(TORQUE_CUT[out_label], OUTPUT_UNIVERSE), alpha)
            aggregated = np.maximum(aggregated, clipped)

    # 4) Defuzzification
    area = np.trapezoid(aggregated, OUTPUT_UNIVERSE)
    if area == 0:
        cog = 0.0
        mom = 0.0
    else:
        cog = float(np.trapezoid(aggregated * OUTPUT_UNIVERSE, OUTPUT_UNIVERSE) / area)
        top = aggregated.max()
        at_max = OUTPUT_UNIVERSE[np.isclose(aggregated, top, atol=1e-9)]
        mom = float((at_max.min() + at_max.max()) / 2)

    return {"fuzzified": fuzzified, "alphas": alphas, "aggregated": aggregated,
            "cog": cog, "mom": mom}


def print_report(slip, lean, result):
    print(f"\nInput: slip = {slip} %, lean = {lean} deg\n")
    print("Fuzzification")
    for var in ("slip", "lean"):
        degrees = ", ".join(f"{k}={v:.2f}" for k, v in result["fuzzified"][var].items())
        print(f"  {var:<5}: {degrees}")
    print("\nRule firing strengths (min)")
    for name, out_label, alpha in result["alphas"]:
        status = f"-> {out_label}" if alpha > 0 else "(not fired)"
        print(f"  {name}: alpha = {alpha:.2f} {status}")
    print("\nDefuzzification")
    print(f"  Mean of maximum   : {result['mom']:.1f} %")
    print(f"  Centre of gravity : {result['cog']:.1f} %   <- control action\n")


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------
def plot_inference(slip, lean, result, path):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4.2))
    for name, out_label, alpha in result["alphas"]:
        if alpha > 0:
            clipped = np.minimum(membership(TORQUE_CUT[out_label], OUTPUT_UNIVERSE), alpha)
            ax.plot(OUTPUT_UNIVERSE, clipped, lw=2, label=f"{name}: {out_label} (α={alpha:.2f})")
    ax.fill_between(OUTPUT_UNIVERSE, result["aggregated"], alpha=0.15, label="aggregated (max)")
    ax.axvline(result["cog"], color="k", label=f"centre of gravity = {result['cog']:.1f} %")
    ax.axvline(result["mom"], color="k", ls="--", lw=1, label=f"mean of maximum = {result['mom']:.1f} %")
    ax.set_xlabel("torque cut (%)")
    ax.set_ylabel("membership degree")
    ax.set_ylim(0, 1.05)
    ax.set_title(f"Mamdani inference — slip {slip} %, lean {lean}°")
    ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_control_surface(path):
    import matplotlib.pyplot as plt

    slips = np.linspace(0, 20, 81)
    leans = np.linspace(0, 60, 121)
    surface = np.zeros((len(leans), len(slips)))
    for i in range(len(leans)):
        for j in range(len(slips)):
            surface[i, j] = infer(slips[j], leans[i])["cog"]

    fig, ax = plt.subplots(figsize=(7, 5))
    mesh = ax.pcolormesh(slips, leans, surface, shading="auto", cmap="viridis", vmin=0, vmax=100)
    ax.plot(12, 40, "r*", ms=12, label="case study (12 %, 40°) → 55.4 %")
    ax.plot(18, 30, "wx", ms=10, mew=2, label="rule-base gap (18 %, 30°) → 0 %")
    fig.colorbar(mesh, label="torque cut (%)")
    ax.set_xlabel("rear-wheel slip (%)")
    ax.set_ylabel("lean angle (°)")
    ax.set_title("Control surface of the fuzzy traction control")
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Fuzzy traction control (Mamdani)")
    parser.add_argument("--slip", type=float, default=12.0, help="rear-wheel slip in %% (0-20)")
    parser.add_argument("--lean", type=float, default=40.0, help="lean angle in degrees (0-60)")
    parser.add_argument("--plots", action="store_true", help="save figures to images/")
    args = parser.parse_args()

    result = infer(args.slip, args.lean)
    print_report(args.slip, args.lean, result)

    if args.plots:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")
        os.makedirs(out_dir, exist_ok=True)
        plot_inference(args.slip, args.lean, result, os.path.join(out_dir, "inference-python.png"))
        plot_control_surface(os.path.join(out_dir, "control-surface.png"))
        print(f"Figures saved in {out_dir}")


if __name__ == "__main__":
    main()
