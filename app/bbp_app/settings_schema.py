"""Every setting a project has, with its label, range and help text.

There are two scopes:
  * "setup": the experiment. These apply to the simulator AND the real platform
    (target, artificial effects, camera timing, control limits). Shown in the Setup tab.
  * "model": the simulated hardware. These only exist in the simulator (ball size and mass,
    friction, servo characteristics, ...). Shown in Simulation > Simulated hardware.

The runner receives all of them as a `key = value` file (see framework/src/settings.cpp).
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
    presets: dict = field(default_factory=dict)  # choice value -> {key: value} filled in on selection


SETUP_GROUPS = ["Target", "Artificial effects", "Camera", "Control"]
MODEL_GROUPS = ["Ball", "Contact", "Servos", "Plate", "Camera model", "Environment"]
GROUPS = SETUP_GROUPS + MODEL_GROUPS
SCOPE = {g: "setup" for g in SETUP_GROUPS} | {g: "model" for g in MODEL_GROUPS}

BALL_PRESETS = {
    "steel": {"ball.material": "steel", "ball.radius_mm": 10.0, "ball.mass_g": 32.7, "ball.wall_mm": 0.0},
    "glass": {"ball.material": "glass", "ball.radius_mm": 10.0, "ball.mass_g": 10.5, "ball.wall_mm": 0.0},
    "plastic_white": {"ball.material": "plastic_white", "ball.radius_mm": 10.0, "ball.mass_g": 5.0,
                      "ball.wall_mm": 0.0},
    "hollow_orange": {"ball.material": "hollow_orange", "ball.radius_mm": 10.0, "ball.mass_g": 0.75,
                      "ball.wall_mm": 0.6},
}

MATERIALS = [("steel", "Polished steel"), ("glass", "Glass"), ("plastic_white", "White plastic"),
             ("hollow_orange", "Orange plastic")]

SETTINGS = [
    # ---------------------------------------------------------------- setup: Target
    Setting("target.mode", "Target path", "Target", "choice", "fixed",
            choices=[("fixed", "Fixed point"), ("circle", "Circle"), ("square", "Square (steps)"),
                     ("figure8", "Figure eight")],
            help="Where control() is asked to put the ball. You can also set a fixed target live while running."),
    Setting("target.x_mm", "Centre x", "Target", "float", 0.0, -80, 80, 1, "mm"),
    Setting("target.y_mm", "Centre y", "Target", "float", 0.0, -80, 80, 1, "mm"),
    Setting("target.radius_mm", "Radius", "Target", "float", 40.0, 0, 80, 1, "mm"),
    Setting("target.period_s", "Period", "Target", "float", 8.0, 1, 60, 0.5, "s", 1),
    # ---------------------------------------------------------------- setup: Artificial effects
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
    # ---------------------------------------------------------------- setup: Camera timing
    Setting("camera.fps", "Frame rate", "Camera", "float", 30.0, 5, 60, 1, "fps"),
    Setting("camera.exposure_ms", "Exposure time", "Camera", "float", 6.0, 0.5, 30, 0.5, "ms", 1,
            help="Longer exposure: more motion blur on a moving ball."),
    # ---------------------------------------------------------------- setup: Control
    Setting("control.max_angle_deg", "Max plate angle", "Control", "float", 8.0, 1, 15, 0.5, "deg", 1,
            help="Your angles are clamped to this before they are sent to the servos."),
    Setting("control.velocity_filter", "Velocity filter", "Control", "float", 0.5, 0, 0.95, 0.05, "", 2,
            help="Smoothing of ball.vx / ball.vy. 0 = raw finite difference, close to 1 = heavy smoothing."),

    # ---------------------------------------------------------------- model: Ball
    Setting("ball.preset", "Preset", "Ball", "choice", "steel",
            choices=[("steel", "Steel bearing ball, 20 mm"), ("glass", "Glass marble, 20 mm"),
                     ("plastic_white", "White plastic, 20 mm (solid)"),
                     ("hollow_orange", "Orange plastic, 20 mm (hollow)"), ("custom", "Custom")],
            presets=BALL_PRESETS,
            help="Fills in the values below. Change any of them to make your own ball."),
    Setting("ball.material", "Surface material", "Ball", "choice", "steel", choices=MATERIALS,
            help="How the ball looks to the camera, and its stiffness (for contact friction)."),
    Setting("ball.radius_mm", "Radius", "Ball", "float", 10.0, 4, 25, 0.5, "mm", 1),
    Setting("ball.mass_g", "Mass", "Ball", "float", 32.7, 0.1, 500, 0.1, "g", 2),
    Setting("ball.wall_mm", "Wall thickness", "Ball", "float", 0.0, 0, 25, 0.1, "mm", 1,
            help="0 = solid ball. A thin wall (hollow ball) has more rolling inertia, so it accelerates "
                 "more slowly: 3/5 g sin(angle) instead of 5/7 g sin(angle)."),
    Setting("ball.start_x_mm", "Start position x", "Ball", "float", 50.0, -85, 85, 1, "mm"),
    Setting("ball.start_y_mm", "Start position y", "Ball", "float", -30.0, -85, 85, 1, "mm"),
    # ---------------------------------------------------------------- model: Contact
    Setting("contact.static_friction", "Static friction", "Contact", "float", 0.45, 0.02, 1.5, 0.05, "", 2),
    Setting("contact.kinetic_friction", "Sliding friction", "Contact", "float", 0.40, 0.02, 1.5, 0.05, "", 2),
    Setting("contact.rolling_resistance", "Rolling resistance", "Contact", "float", 0.001, 0, 0.05, 0.0005, "", 4,
            help="Causes a dead zone: below about atan(value) the ball does not start rolling."),
    Setting("contact.restitution", "Bounciness (restitution)", "Contact", "float", 0.5, 0, 0.95, 0.05, "", 2),
    # ---------------------------------------------------------------- model: Servos
    Setting("servo.stall_torque", "Stall torque", "Servos", "float", 1.08, 0.05, 5, 0.05, "N m", 2),
    Setting("servo.speed_s_per_60", "Speed", "Servos", "float", 0.14, 0.03, 1.0, 0.01, "s/60 deg", 2,
            help="Datasheet speed: time to turn 60 degrees without load."),
    Setting("servo.time_constant_ms", "Mechanical time constant", "Servos", "float", 20.0, 2, 200, 1, "ms",
            help="How quickly the motor reaches speed (sets the motor and gear inertia)."),
    Setting("servo.deadband_us", "Deadband", "Servos", "float", 5.0, 0, 50, 1, "us",
            help="Pulse-width change the servo ignores."),
    Setting("servo.prop_band_deg", "Proportional band", "Servos", "float", 6.0, 0.5, 45, 0.5, "deg", 1,
            help="Error at which the servo's internal controller drives at full power."),
    Setting("servo.control_rate_hz", "Internal update rate", "Servos", "float", 250.0, 20, 2000, 10, "Hz"),
    Setting("servo.pot_noise_deg", "Position sensor noise", "Servos", "float", 0.05, 0, 2, 0.01, "deg", 2),
    # ---------------------------------------------------------------- model: Plate
    Setting("plate.mass_g", "Plate mass", "Plate", "float", 188.0, 20, 2000, 1, "g",
            help="Default: the CAD plate in acrylic (PMMA)."),
    Setting("plate.refractive_index", "Refractive index", "Plate", "float", 1.49, 1.0, 2.0, 0.01, "", 2,
            help="The camera looks through the plate; refraction shifts the image of the ball."),
    # ---------------------------------------------------------------- model: Camera
    Setting("camera.noise", "Image noise", "Camera model", "float", 1.0, 0, 5, 0.1, "x", 1,
            help="Scales the sensor noise. 0 = noise-free images."),
    Setting("camera.ring_light", "Ring light at camera", "Camera model", "bool", False,
            help="Adds a light next to the camera, which makes the ball's underside visible."),
    Setting("camera.hfov_deg", "Lens field of view", "Camera model", "float", 90.0, 40, 130, 1, "deg",
            help="Horizontal field of view of the undistorted lens model."),
    Setting("camera.k1", "Lens distortion k1", "Camera model", "float", -0.25, -0.6, 0.3, 0.01, "", 2,
            help="Radial (barrel) distortion. Negative = barrel."),
    # ---------------------------------------------------------------- model: Environment
    Setting("env.gravity", "Gravity", "Environment", "float", 9.80665, 0.5, 30, 0.1, "m/s^2", 3,
            help="Try the Moon (1.62) or Mars (3.71)."),
    Setting("env.air_drag", "Air drag", "Environment", "bool", True),
    Setting("sim.seed", "Random seed", "Environment", "int", 1, 0, 1_000_000, 1,
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
