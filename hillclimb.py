import pygame
import noise
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Terrain properties
TERRAIN_WIDTH = WIDTH
TERRAIN_RESOLUTION = 2
AMPLITUDE = 120
FREQUENCY = 0.005

# Vehicle properties
CAR_WIDTH = 100
CAR_HEIGHT = 40
WHEEL_RADIUS = 20
SUSPENSION_STIFFNESS = 0.2
SUSPENSION_DAMPING = 0.1
GRAVITY = 0.5

# Vehicle state
car_x = WIDTH // 4
car_y = HEIGHT // 2
car_velocity = 0
car_dy = 0
car_angle = 0  # Rotation angle in radians

# Suspension system (two wheels)
wheel_offsets = [-CAR_WIDTH // 2 + WHEEL_RADIUS, CAR_WIDTH // 2 - WHEEL_RADIUS]
wheels_y = [car_y + CAR_HEIGHT + WHEEL_RADIUS] * len(wheel_offsets)
wheel_velocities = [0] * len(wheel_offsets)

# Terrain generation
def generate_terrain(offset):
    terrain = []
    for i in range(0, TERRAIN_WIDTH, TERRAIN_RESOLUTION):
        x = (i + offset) * FREQUENCY
        height = HEIGHT // 2 + noise.pnoise1(x) * AMPLITUDE
        terrain.append((i, HEIGHT - height))
    return terrain

# Draw filled terrain
def draw_terrain(terrain):
    # Create a list of points for the terrain polygon
    points = [(0, HEIGHT)]  # Start with the bottom left corner
    points.extend(terrain)  # Add terrain points
    points.append((TERRAIN_WIDTH, HEIGHT))  # End with the bottom right corner
    pygame.draw.polygon(screen, (100, 200, 100), points)  # Fill with a greenish color

# Get terrain height at a specific x-coordinate
def get_terrain_height(terrain, x):
    x_index = x // TERRAIN_RESOLUTION
    if 0 <= x_index < len(terrain) - 1:
        t1, t2 = terrain[x_index], terrain[x_index + 1]
        slope = (t2[1] - t1[1]) / (t2[0] - t1[0])
        return t1[1] + slope * (x % TERRAIN_RESOLUTION)
    return HEIGHT  # Default to flat ground

# Collision detection: Check if a wheel is colliding with the ground (terrain)
def check_collision_with_terrain(wheel_x, wheel_y, terrain):
    # Find the terrain height at the wheel's x-coordinate
    terrain_height = get_terrain_height(terrain, wheel_x)
    # Check if the wheel's bottom is below the terrain
    return wheel_y + WHEEL_RADIUS > terrain_height

# Vehicle physics and suspension
def move_vehicle(car_y, car_dy, wheels_y, wheel_velocities, terrain):
    keys = pygame.key.get_pressed()
    global car_velocity, car_angle

    # Forward acceleration
    if keys[pygame.K_RIGHT]:
        car_velocity = min(car_velocity + 0.1, 5)
    else:
        car_velocity = max(car_velocity - 0.05, 0)

    # Suspension forces and wheel dynamics
    wheel_positions = []
    for i, wheel_offset in enumerate(wheel_offsets):
        wheel_x = car_x + wheel_offset
        terrain_height = get_terrain_height(terrain, wheel_x)

        # Calculate spring compression (allowing wheels to go above the terrain)
        compression = wheels_y[i] - terrain_height - WHEEL_RADIUS
        spring_force = -SUSPENSION_STIFFNESS * compression
        damping_force = -SUSPENSION_DAMPING * wheel_velocities[i]

        # Update wheel vertical velocity and position
        wheel_velocities[i] += GRAVITY + spring_force + damping_force
        wheels_y[i] += wheel_velocities[i]

        # Check collision: If the wheel is below the terrain, stop it at the terrain height
        if check_collision_with_terrain(wheel_x, wheels_y[i], terrain):
            wheels_y[i] = terrain_height - WHEEL_RADIUS
            wheel_velocities[i] = 0  # Stop the downward motion

        wheel_positions.append((car_x + wheel_offset, wheels_y[i]))

    # Adjust car position and angle based on wheels
    left_wheel, right_wheel = wheel_positions
    dx = right_wheel[0] - left_wheel[0]
    dy = right_wheel[1] - left_wheel[1]
    car_angle = math.atan2(dy, dx)  # Angle based on terrain slope

    # Car's center follows the midpoint between wheels
    car_y = (left_wheel[1] + right_wheel[1]) / 2 - CAR_HEIGHT - WHEEL_RADIUS

    return car_y, wheels_y, wheel_velocities, car_angle

# Draw the car with rotation
def draw_car(car_y, wheels_y, car_angle):
    # Draw wheels (now they have hitboxes and are constrained above the terrain)
    for i, wheel_offset in enumerate(wheel_offsets):
        wheel_x = car_x + wheel_offset
        pygame.draw.circle(screen, (0, 0, 0), (wheel_x, int(wheels_y[i])), WHEEL_RADIUS)

    # Draw car body with rotation
    car_center = (car_x, car_y + CAR_HEIGHT // 2)
    car_surface = pygame.Surface((CAR_WIDTH, CAR_HEIGHT), pygame.SRCALPHA)
    car_surface.fill((255, 0, 0))
    rotated_car = pygame.transform.rotate(car_surface, -math.degrees(car_angle))
    car_rect = rotated_car.get_rect(center=car_center)
    screen.blit(rotated_car, car_rect.topleft)

# Main game loop
terrain_offset = 0
running = True

while running:
    screen.fill((135, 206, 235))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Generate and draw terrain
    terrain = generate_terrain(terrain_offset)
    draw_terrain(terrain)

    # Move and draw the car
    car_y, wheels_y, wheel_velocities, car_angle = move_vehicle(
        car_y, car_dy, wheels_y, wheel_velocities, terrain
    )
    draw_car(car_y, wheels_y, car_angle)

    # Scroll terrain
    terrain_offset += car_velocity * TERRAIN_RESOLUTION

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
