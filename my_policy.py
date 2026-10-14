import math


ALTITUDE = 14.0
SPEED = 15.0
DEDUP_RADIUS = 10.0
PRIOR_MASS_TARGET = 0.90


class Policy:
    def __init__(self, info):
        self.arena_size = info["arena_size"]
        self.grid_size = info["grid_size"]
        self.cell_size = info["cell_size"]
        self.energy_budget = info["energy_budget"]
        footprint_radius = ALTITUDE * info.get("footprint_tan", math.tan(math.radians(40.0)))
        self.waypoints = self._plan_search(info["prior_map"], footprint_radius, info["base_xy"])
        self.waypoint_index = 0
        self.declared_positions = []
        self.pending_declarations = []
        self.returning_home = False
        self.estimated_power = 2.0
        self.last_action = None

    def _plan_search(self, prior_map, footprint_radius, base_xy):
        probabilities = [float(value) for row in prior_map for value in row]
        remaining = probabilities[:]
        prior_total = sum(remaining)
        if prior_total <= 0.0:
            return self._fallback_waypoints()

        centers = [
            ((cell_x + 0.5) * self.cell_size, (cell_y + 0.5) * self.cell_size)
            for cell_x in range(self.grid_size)
            for cell_y in range(self.grid_size)
        ]
        radius_cells = int(math.ceil(footprint_radius / self.cell_size))
        coverage = []
        for cell_x in range(self.grid_size):
            for cell_y in range(self.grid_size):
                covered = []
                for other_x in range(max(0, cell_x - radius_cells), min(self.grid_size, cell_x + radius_cells + 1)):
                    for other_y in range(max(0, cell_y - radius_cells), min(self.grid_size, cell_y + radius_cells + 1)):
                        dx = (cell_x - other_x) * self.cell_size
                        dy = (cell_y - other_y) * self.cell_size
                        if dx * dx + dy * dy <= footprint_radius * footprint_radius:
                            covered.append(other_x * self.grid_size + other_y)
                coverage.append(covered)

        targets = []
        uncovered_mass = prior_total
        mass_limit = prior_total * (1.0 - PRIOR_MASS_TARGET)
        while uncovered_mass > mass_limit:
            best_index = -1
            best_gain = 0.0
            for center_index, covered in enumerate(coverage):
                gain = sum(remaining[cell_index] for cell_index in covered)
                if gain > best_gain:
                    best_index = center_index
                    best_gain = gain
            if best_index < 0 or best_gain <= 0.0:
                break
            targets.append(centers[best_index])
            for cell_index in coverage[best_index]:
                remaining[cell_index] = 0.0
            uncovered_mass -= best_gain

        ordered = []
        unvisited = set(range(len(targets)))
        current_x, current_y = base_xy
        while unvisited:
            next_index = min(
                unvisited,
                key=lambda index: math.hypot(targets[index][0] - current_x, targets[index][1] - current_y),
            )
            current_x, current_y = targets[next_index]
            ordered.append(targets[next_index])
            unvisited.remove(next_index)
        return ordered or self._fallback_waypoints()

    def _fallback_waypoints(self):
        lane_spacing = 1.6 * ALTITUDE * math.tan(math.radians(40.0))
        waypoints = []
        lane_x = lane_spacing / 2.0
        sweep_up = True
        while lane_x < self.arena_size:
            start_y, end_y = (0.0, self.arena_size) if sweep_up else (self.arena_size, 0.0)
            waypoints.append((lane_x, start_y))
            waypoints.append((lane_x, end_y))
            lane_x += lane_spacing
            sweep_up = not sweep_up
        return waypoints

    def act(self, obs):
        drone_x, drone_y = obs["x"], obs["y"]
        if self.last_action is not None and self.last_action[0] == "fly_to" and obs["t"] > self.last_action[1]:
            elapsed = obs["t"] - self.last_action[1]
            observed_power = obs["energy_used_last_action"] / elapsed
            self.estimated_power = 0.7 * self.estimated_power + 0.3 * observed_power

        for detection in obs["detections"]:
            is_new = all(
                math.hypot(detection["x"] - declared_x, detection["y"] - declared_y) > DEDUP_RADIUS
                for declared_x, declared_y, _ in self.declared_positions
            )
            if is_new:
                position = (detection["x"], detection["y"], detection.get("triage"))
                self.declared_positions.append(position)
                self.pending_declarations.append(position)

        if self.pending_declarations:
            survivor_x, survivor_y, detected_triage = self.pending_declarations.pop(0)
            self.last_action = ("declare", obs["t"])
            return {
                "type": "declare",
                "x": survivor_x,
                "y": survivor_y,
                "triage": detected_triage or "serious",
            }

        distance_home = math.hypot(drone_x, drone_y)
        if not self.returning_home:
            return_energy_estimate = distance_home / SPEED * self.estimated_power * 1.5 + 0.05 * self.energy_budget
            if obs["energy_remaining"] < return_energy_estimate or self.waypoint_index >= len(self.waypoints):
                self.returning_home = True

        if self.returning_home:
            if distance_home <= 10.0:
                return {"type": "land"}
            self.last_action = ("fly_to", obs["t"])
            return {"type": "fly_to", "x": 0.0, "y": 0.0, "altitude": ALTITUDE, "speed": SPEED}

        target_x, target_y = self.waypoints[self.waypoint_index]
        self.waypoint_index += 1
        self.last_action = ("fly_to", obs["t"])
        return {"type": "fly_to", "x": target_x, "y": target_y, "altitude": ALTITUDE, "speed": SPEED}