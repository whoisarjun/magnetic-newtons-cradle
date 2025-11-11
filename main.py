import sys
import math
import os
import magnets
import pygame


# ---------- Simulation Parameters ----------
WIDTH, HEIGHT = 900, 600
FPS = 120

# ---------- Recording Configuration ----------
RECORD_VIDEO = True           # Set to False to disable frame capture
VIDEO_DURATION = 300.0        # seconds (5 minutes)
FRAME_OUTPUT_DIR = "frames"   # directory to save frames

NUM_BALLS = 5
ROPE_LENGTH = 250
BALL_RADIUS = 20
GRAVITY = 900  # pixels/s^2, tuned for nice motion

ANCHOR_Y = 100
BALL_SPACING = BALL_RADIUS * 3  # distance between anchors horizontally

BACKGROUND_COLOR = (10, 10, 15)
ANCHOR_COLOR = (220, 220, 220)
ROPE_COLOR = (200, 200, 200)
BALL_COLOR = (230, 230, 255)

MAGNETS = magnets.load_magnets_from_csv('magnets.csv')

class NewtonsCradle:
    def __init__(self, screen):
        self.screen = screen

        # Equally spaced anchors centered horizontally
        start_x = WIDTH // 2 - (NUM_BALLS - 1) * BALL_SPACING // 2
        self.anchors = [
            (start_x + i * BALL_SPACING, ANCHOR_Y) for i in range(NUM_BALLS)
        ]

        # Angles (θ) from vertical (radians). Positive = to the right.
        self.angles = [0.0 for _ in range(NUM_BALLS)]
        # Angular velocities (ω)
        self.angular_velocities = [0.0 for _ in range(NUM_BALLS)]

        # Dragging state for the first ball
        self.dragging_first = False

        # Fonts
        self.font_main = pygame.font.SysFont("SF Pro Text", 18)
        self.font_small = pygame.font.SysFont("SF Pro Text", 14)

    # ---------- Physics Helpers ----------

    def get_ball_position(self, index):
        """Compute (x, y) of ball i from its angle θ."""
        ax, ay = self.anchors[index]
        theta = self.angles[index]
        x = ax + ROPE_LENGTH * math.sin(theta)
        y = ay + ROPE_LENGTH * math.cos(theta)
        return x, y

    def get_ball_horizontal_velocity(self, index):
        """Approximate horizontal velocity from angular velocity."""
        theta = self.angles[index]
        omega = self.angular_velocities[index]
        # Tangential speed = L * ω, horizontal component = that * cos(theta)
        return ROPE_LENGTH * omega * math.cos(theta)

    def set_omega_from_horizontal_velocity(self, index, v_x):
        """Approximate ω from horizontal velocity."""
        theta = self.angles[index]
        denom = ROPE_LENGTH * max(math.cos(theta), 0.1)
        self.angular_velocities[index] = v_x / denom

    def get_force_info(self, index):
        """
        Compute effective tangential force driving motion (unit mass):
        F_t = -g * sin(theta) along the pendulum's tangent.
        Returns (magnitude, direction_angle_radians), where direction is
        the global angle of the force vector using atan2(F_y, F_x).
        """
        theta = self.angles[index]

        # Tangential force magnitude (unit mass)
        Ft = -GRAVITY * math.sin(theta)

        # Tangent unit vector for increasing theta
        tx = math.cos(theta)
        ty = -math.sin(theta)

        # Force vector components
        Fx = Ft * tx
        Fy = Ft * ty

        mag = abs(Ft)
        angle = math.atan2(Fy, Fx) if mag > 0 else 0.0

        return mag, angle

    # ---------- Event Handling ----------

    def handle_events(self):
        mouse_pos = pygame.mouse.get_pos()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # Start dragging first ball
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                first_x, first_y = self.get_ball_position(0)
                dx = mouse_pos[0] - first_x
                dy = mouse_pos[1] - first_y
                if dx * dx + dy * dy <= (BALL_RADIUS * 2.5) ** 2:
                    self.dragging_first = True

            # Stop dragging, release into motion
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if self.dragging_first:
                    self.dragging_first = False
                    # On release, keep the angle implied by last drag,
                    # but start with zero angular velocity (pendulum will swing from rest)
                    self.angular_velocities[0] = 0.0

        # While dragging, continuously set first ball angle from mouse position
        if self.dragging_first:
            ax, ay = self.anchors[0]
            mx, my = mouse_pos

            # Vector from anchor to mouse
            dx = mx - ax
            dy = my - ay

            # Prevent degenerate (very short) rope while dragging
            length = math.hypot(dx, dy)
            if length < 10:
                return

            # Angle from vertical: θ = atan2(x, y)
            theta = math.atan2(dx, dy)

            # Do not allow the first ball to go past the vertical (to the right side),
            # since that breaks the cradle behavior. Clamp to θ ≤ 0.
            if theta > 0:
                theta = 0.0

            self.angles[0] = theta
            self.angular_velocities[0] = 0.0  # freeze velocity while dragging

    # ---------- Physics Update ----------

    def step_physics(self, dt):
        i_sets = [(0, 1), (1, 2), (2, 3), (3, 4)]
        forces = []
        for m, n in i_sets:
            # Use full 2D distance between ball centers
            mx, my = self.get_ball_position(m)
            nx, ny = self.get_ball_position(n)
            dx = nx - mx
            dy = ny - my
            dist = math.hypot(dx, dy)

            if dist <= 2 * BALL_RADIUS:
                # Already in contact (or overlapping) -> treat via collision only,
                # so do NOT apply magnetic force here.
                rep = 0.0
            else:
                # Apply magnetic force only when separated
                rep = magnets.force_between(
                    MAGNETS[m],
                    (0.0, 0.0, mx),
                    magnets.R_identity,
                    MAGNETS[n],
                    (0.0, 0.0, nx),
                    magnets.R_identity
                )[0][-1]

            forces.append(rep)

        # Skip physics for first ball angle while dragging
        for i in range(NUM_BALLS):
            if i == 0 and self.dragging_first:
                continue

            theta = self.angles[i]
            omega = self.angular_velocities[i]

            # Position of this ball
            ax, ay = self.anchors[i]
            x, y = self.get_ball_position(i)

            # Rope direction (anchor -> ball)
            rx = x - ax
            ry = y - ay
            length = math.hypot(rx, ry)
            if length < 1e-8:
                continue
            ux = rx / length
            uy = ry / length

            # Tangent direction along the arc (for increasing theta)
            tx = uy
            ty = -ux

            # Resultant force (for now only weight, downward)
            if i == 0:
                Fx = -forces[0]
            elif i == 1:
                Fx = forces[0] - forces[1]
            elif i == 2:
                Fx = forces[1] - forces[2]
            elif i == 3:
                Fx = forces[2] - forces[3]
            else:
                Fx = forces[3]

            Fx *= 200

            Fy = GRAVITY

            # Tangential component of resultant force
            Ft = Fx * tx + Fy * ty

            # Angular acceleration from tangential force:
            # For a point mass m at radius L: alpha = Ft / (m * L), with m = 1.
            alpha = Ft / ROPE_LENGTH

            # Integrate (semi-implicit Euler for stability)
            omega += alpha * dt
            theta += omega * dt

            self.angular_velocities[i] = omega
            self.angles[i] = theta

        # Handle collisions between balls (adjacent, equal-mass, elastic, 1D along x)
        self.handle_collisions()

    def handle_collisions(self):
        """
        Handle instantaneous, perfectly elastic collisions between adjacent balls.

        Model:
        - Each ball is an equal mass constrained to move on its circular arc.
        - Collisions are treated as 1D along the horizontal line through the centers.
        - For equal masses + elastic collision: they exchange their horizontal velocities.
        - We:
            1. Detect contact.
            2. Check if they are closing (so we don't react on separating pairs).
            3. Swap horizontal velocity components.
            4. Map those velocities back into angular velocities respecting the pendulum constraint.
        """
        # Current positions and horizontal velocities
        positions = [self.get_ball_position(i) for i in range(NUM_BALLS)]
        vxs = [self.get_ball_horizontal_velocity(i) for i in range(NUM_BALLS)]

        for i in range(NUM_BALLS - 1):
            x1, y1 = positions[i]
            x2, y2 = positions[i + 1]

            # Horizontal gap between centers
            dx = x2 - x1

            # Check for overlap/contact
            if dx < 2 * BALL_RADIUS:
                # Relative motion: if left ball is moving right faster than right ball,
                # they are closing and we should collide.
                if vxs[i] > vxs[i + 1]:
                    # Equal-mass, 1D, elastic: swap velocities
                    v1_new = vxs[i + 1]
                    v2_new = vxs[i]

                    # Apply back to angular velocities based on each ball's current angle
                    self.set_omega_from_horizontal_velocity(i, v1_new)
                    self.set_omega_from_horizontal_velocity(i + 1, v2_new)

                    # Positional correction so they are exactly touching (no interpenetration).
                    # This is a geometric constraint enforcement, not "cheating the pattern".
                    mid_x = 0.5 * (x1 + x2)
                    x1_corr = mid_x - BALL_RADIUS
                    x2_corr = mid_x + BALL_RADIUS

                    # Recompute angles from corrected positions to keep rope length + anchor constraint.
                    ax1, ay1 = self.anchors[i]
                    ax2, ay2 = self.anchors[i + 1]

                    dx1 = x1_corr - ax1
                    dy1 = y1 - ay1
                    dx2 = x2_corr - ax2
                    dy2 = y2 - ay2

                    if math.hypot(dx1, dy1) > 1e-8:
                        self.angles[i] = math.atan2(dx1, dy1)
                    if math.hypot(dx2, dy2) > 1e-8:
                        self.angles[i + 1] = math.atan2(dx2, dy2)

    # ---------- Rendering ----------

    def draw(self, fps):
        self.screen.fill(BACKGROUND_COLOR)

        # Draw anchor bar
        pygame.draw.line(
            self.screen,
            ANCHOR_COLOR,
            (self.anchors[0][0] - 40, ANCHOR_Y),
            (self.anchors[-1][0] + 40, ANCHOR_Y),
            4,
        )

        # Draw each rope + ball
        for i in range(NUM_BALLS):
            ax, ay = self.anchors[i]
            x, y = self.get_ball_position(i)

            # Rope
            pygame.draw.line(self.screen, ROPE_COLOR, (ax, ay), (x, y), 2)

            # Ball as rectangle with left half blue and right half red
            rect_width = BALL_RADIUS * 2
            rect_height = BALL_RADIUS * 2
            rect_x = int(x - BALL_RADIUS)
            rect_y = int(y - BALL_RADIUS)

            left_rect = pygame.Rect(rect_x, rect_y, rect_width // 2, rect_height)
            right_rect = pygame.Rect(rect_x + rect_width // 2, rect_y, rect_width // 2, rect_height)

            if i % 2:
                left_rect, right_rect = right_rect, left_rect

            pygame.draw.rect(self.screen, (70, 100, 255), left_rect)
            pygame.draw.rect(self.screen, (255, 70, 70), right_rect)

        # Lightweight hint text
        text = self.font_main.render(
            "Drag the leftmost ball and release.",
            True,
            (180, 180, 200),
        )
        self.screen.blit(text, (20, 20))

        # FPS counter (bottom-left)
        fps_text = self.font_main.render(
            f"FPS: {fps:.0f}",
            True,
            (180, 180, 200),
        )
        fps_rect = fps_text.get_rect()
        fps_rect.topleft = (20, HEIGHT - fps_rect.height - 10)
        self.screen.blit(fps_text, fps_rect)

        # Force info under each ball (static labels under each anchor, split into 2 lines)
        for i in range(NUM_BALLS):
            mag, angle = self.get_force_info(i)
            force_text = f"F={mag:5.1f}"
            theta_text = f"θ={angle:+.2f}"

            ax, _ = self.anchors[i]

            # First line: force magnitude
            force_surf = self.font_small.render(
                force_text,
                True,
                (180, 180, 200),
            )
            force_rect = force_surf.get_rect()
            force_rect.centerx = ax
            force_rect.top = HEIGHT - 2 * force_rect.height - 14
            self.screen.blit(force_surf, force_rect)

            # Second line: angle
            theta_surf = self.font_small.render(
                theta_text,
                True,
                (180, 180, 200),
            )
            theta_rect = theta_surf.get_rect()
            theta_rect.centerx = ax
            theta_rect.top = HEIGHT - theta_rect.height - 8
            self.screen.blit(theta_surf, theta_rect)


def main():
    pygame.init()
    pygame.display.set_caption("Newton's Cradle Simulation")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()

    if RECORD_VIDEO:
        os.makedirs(FRAME_OUTPUT_DIR, exist_ok=True)
    elapsed = 0.0
    frame_index = 0

    cradle = NewtonsCradle(screen)

    while True:
        dt = clock.tick(FPS) / 1000.0  # seconds per frame
        elapsed += dt

        cradle.handle_events()
        cradle.step_physics(dt)
        cradle.draw(clock.get_fps())

        pygame.display.flip()

        if RECORD_VIDEO:
            frame_path = os.path.join(FRAME_OUTPUT_DIR, f"frame_{frame_index:05d}.png")
            pygame.image.save(screen, frame_path)
            frame_index += 1

            if elapsed >= VIDEO_DURATION:
                break

    pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
