# Copy to <rollnumber>.py. Interface and rules: README.md


class Policy:
    def __init__(self, info: dict):
        # called once per episode
        self.info = info
        self.steps = 0

    def act(self, obs: dict) -> dict:
        # called before every action; return exactly one action dict
        self.steps += 1
        if self.steps == 1:
            return {"type": "fly_to", "x": 0.0, "y": 0.0, "altitude": 20.0, "speed": 5.0}
        if self.steps == 2:
            return {"type": "hover", "duration": 2.0}
        return {"type": "land"}
