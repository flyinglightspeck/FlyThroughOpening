class CollisionTracker:
    def __init__(self, level=0):
        self.history_collisions = []
        self.active_collisions = []
        self.collision_id_counter = -1
        self.level = level

    def _generate_collision_id(self):
        self.collision_id_counter += 1
        return self.collision_id_counter

    def update_collisions(self, current_collisions):
        """
        Update the tracker with the list of current collisions.
        Each collision is represented as a set of drone IDs.
        """
        new_active_collisions = []

        for collision in current_collisions:
            # Check if this collision matches an existing one
            matched = False


            if self.level <= 1:
                for collision_id, _, active_collision in self.active_collisions:
                    if self.level == 0 and collision == active_collision:
                        new_active_collisions.append((collision_id, 1, collision))  # 1 for existing collision
                        matched = True
                        break
                    elif self.level == 1 and collision & active_collision:  # If there is any intersection, treat it as the same collision
                        new_active_collisions.append((collision_id, 1, collision | active_collision))
                        matched = True
                        break
            elif self.level == 2:
                for collision_id, history_collision in self.history_collisions:
                    if collision & history_collision:  # If there is any intersection, treat it as the same collision
                        new_active_collisions.append((collision_id, 1, collision | history_collision))

                        self.history_collisions[collision_id] = (collision_id, collision | history_collision)
                        matched = True
                        break

            if not matched:
                # New collision, assign a new ID
                new_collision_id = self._generate_collision_id()
                new_active_collisions.append((new_collision_id, 0, collision))  # 0 for new collision
                if self.level == 2:
                    self.history_collisions.append((new_collision_id, collision))

        # Update active collisions
        self.active_collisions = new_active_collisions
        return self.active_collisions

    def get_active_collisions(self):
        return self.active_collisions

    def get_collision_count(self):
        return self.collision_id_counter + 1

