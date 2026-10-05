"""The Setup tab: every setting a project has, with its label, range and help text.

The runner receives them as a `key = value` file (see framework/src/settings.cpp).
"""
from dataclasses import dataclass, field


@dataclass
class Setting:
    key: str
    label: str
    group: str
    kind: str                 # "float", "int", "bool", "choice"
    default: object
    minimum: float = 0.0
    maximum: float = 1.0
    step: float = 1.0
    unit: str = ""
    decimals: int = 0
    choices: list = field(default_factory=list)  # [(value, label)]
    help: str = ""


GROUPS = ["Ball", "Target", "Artificial effects", "Camera", "Control", "Physics", "Simulation"]

SETTINGS = [
    # Ball
    Setting("ball.material", "Ball", "Ball", "choice", "steel",
            choices=[("steel", "Steel, 20 mm (solid)"), ("glass", "Glass marble, 20 mm"),
                     ("plastic_white", "White plastic, 20 mm (solid)"),
                     ("hollow_orange", "Orange plastic, 20 mm (hollow)")],
            help="Changes the mass, the rolling inertia (solid vs hollow) and how the ball looks to the camera."),
    Setting("ball.start_x_mm", "Start position x", "Ball", "float", 50.0, -85, 85, 1, "mm"),
    Setting("ball.start_y_mm", "Start position y", "Ball", "float", -30.0, -85, 85, 1, "mm"),
    # Target
    Setting("target.mode", "Target path", "Target", "choice", "fixed",
            choices=[("fixed", "Fixed point"), ("circle", "Circle"), ("square", "Square (steps)"),
                     ("figure8", "Figure eight")],
            help="Where control() is asked to put the ball. You can also set a fixed target live while running."),
    Setting("target.x_mm", "Centre x", "Target", "float", 0.0, -80, 80, 1, "mm"),
    Setting("target.y_mm", "Centre y", "Target", "float", 0.0, -80, 80, 1, "mm"),
    Setting("target.radius_mm", "Radius", "Target", "float", 40.0, 0, 80, 1, "mm"),
    Setting("target.period_s", "Period", "Target", "float", 8.0, 1, 60, 0.5, "s", 1),
    # Artificial effects
    Setting("effects.sensor_delay_ms", "Sensor delay", "Artificial effects", "float", 0.0, 0, 500, 5, "ms",
            help="Extra delay before each camera frame reaches track()."),
    Setting("effects.fps_cap", "Frame-rate cap", "Artificial effects", "float", 0.0, 0, 60, 1, "fps",
            help="Drop frames so track() runs at most this often. 0 = no cap."),
    Setting("effects.motor_delay_ms", "Motor delay", "Artificial effects", "float", 0.0, 0, 500, 5, "ms",
            help="Extra delay between control() and the servos receiving the command."),
    Setting("effects.position_noise_mm", "Position noise", "Artificial effects", "float", 0.0, 0, 10, 0.1, "mm", 1,
            help="Random error (standard deviation) added to the measured ball position."),
    Setting("effects.cpu_model", "Model FireBeetle run time", "Artificial effects", "bool", True,
            help="Delay outputs by the time your code would take on the FireBeetle (PC time x factor)."),
    Setting("effects.cpu_factor", "FireBeetle slowdown factor", "Artificial effects", "float", 40.0, 1, 500, 1, "x",
            help="How much slower the FireBeetle runs your code than this PC. Assumed value; to be calibrated."),
    # Camera
    Setting("camera.fps", "Frame rate", "Camera", "float", 30.0, 5, 60, 1, "fps"),
    Setting("camera.exposure_ms", "Exposure time", "Camera", "float", 6.0, 0.5, 30, 0.5, "ms", 1,
            help="Longer exposure: more motion blur on a moving ball."),
    Setting("camera.noise", "Image noise", "Camera", "float", 1.0, 0, 5, 0.1, "x", 1,
            help="Scales the sensor noise. 0 = noise-free images."),
    Setting("camera.ring_light", "Ring light at camera", "Camera", "bool", False,
            help="Adds a light next to the camera, which makes the ball's underside visible."),
    # Control
    Setting("control.max_angle_deg", "Max plate angle", "Control", "float", 8.0, 1, 15, 0.5, "deg", 1,
            help="Your angles are clamped to this before they are sent to the servos."),
    Setting("control.velocity_filter", "Velocity filter", "Control", "float", 0.5, 0, 0.95, 0.05, "", 2,
            help="Smoothing of ball.vx / ball.vy. 0 = raw finite difference, close to 1 = heavy smoothing."),
    # Physics
    Setting("physics.rolling_resistance", "Rolling resistance", "Physics", "float", 0.001, 0, 0.02, 0.0005, "", 4,
            help="Causes a small dead zone: below about atan(c) degrees the ball does not start rolling."),
    Setting("physics.friction", "Friction coefficient", "Physics", "float", 0.45, 0.05, 1.0, 0.05, "", 2),
    # Simulation
    Setting("sim.speed", "Simulation speed", "Simulation", "float", 1.0, 0.1, 4, 0.1, "x", 1),
    Setting("sim.seed", "Random seed", "Simulation", "int", 1, 0, 1_000_000, 1,
            help="Same seed + same code = exactly the same run."),
]

BY_KEY = {s.key: s for s in SETTINGS}


def defaults():
    return {s.key: s.default for s in SETTINGS}


def write_settings_file(values, path):
    lines = ["# generated by the BBP app"]
    for s in SETTINGS:
        v = values.get(s.key, s.default)
        if s.kind == "bool":
            v = 1 if v else 0
        lines.append(f"{s.key} = {v}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
