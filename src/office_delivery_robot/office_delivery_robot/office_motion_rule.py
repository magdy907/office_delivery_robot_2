def command_speed(travelled: float, target: float, max_speed: float,
                  kp: float = 1.2, tolerance: float = 0.02) -> float:
    remaining = target - travelled
    if remaining <= tolerance:
        return 0.0
    proportional_speed = kp * remaining
    return float(min(max_speed, max(0.04, proportional_speed)))
