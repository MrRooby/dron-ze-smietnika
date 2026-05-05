import pygame
import serial
import re
import math

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM2' 
BAUD_RATE = 115200
WIDTH, HEIGHT = 400, 400
CENTER = (WIDTH // 2, HEIGHT // 2)

# Colors
SKY_BLUE = (110, 180, 230)
GROUND_BROWN = (150, 100, 50)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
YELLOW = (255, 255, 0)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except:
        print("Serial port not found.")
        return

    roll, pitch, yaw = 0, 0, 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        # Parse Serial
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            match = re.search(r"Roll:\s*([-0-9.]+)\s*\|\s*Pitch:\s*([-0-9.]+)\s*\|\s*Yaw:\s*([-0-9.]+)", line)
            if match:
                roll = float(match.group(1))
                pitch = float(match.group(2))
                yaw = float(match.group(3))

        screen.fill(BLACK)

        # 1. Create a surface for the moving horizon
        # Make it larger than the window so edges don't show during rotation
        horizon_surf = pygame.Surface((WIDTH * 2, HEIGHT * 2))
        
        # Pitch offset: Move the split line up/down
        # 1 degree = approx 2 pixels (adjust to taste)
        pitch_offset = pitch * 2 
        
        # Draw Sky and Ground on the surface
        pygame.draw.rect(horizon_surf, SKY_BLUE, (0, 0, WIDTH * 2, HEIGHT + pitch_offset))
        pygame.draw.rect(horizon_surf, GROUND_BROWN, (0, HEIGHT + pitch_offset, WIDTH * 2, HEIGHT * 2))
        
        # Draw white pitch lines
        for i in range(-90, 91, 10):
            y_pos = HEIGHT + pitch_offset - (i * 2)
            pygame.draw.line(horizon_surf, WHITE, (WIDTH - 40, y_pos), (WIDTH + 40, y_pos), 2)

        # 2. Rotate the entire horizon surface based on Roll
        # Negative roll because the "world" rotates opposite to the plane
        rotated_surf = pygame.transform.rotate(horizon_surf, -roll)
        rot_rect = rotated_surf.get_rect(center=CENTER)

        # 3. Clip the rotated surface to a circle (the gauge face)
        gauge_mask = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.circle(gauge_mask, (255, 255, 255, 255), CENTER, 180)
        
        screen.blit(rotated_surf, rot_rect)
        
        # 4. Draw static "Airplane" reference (the yellow wings)
        # Center Dot
        pygame.draw.circle(screen, YELLOW, CENTER, 5)
        # Left Wing
        pygame.draw.rect(screen, YELLOW, (CENTER[0] - 100, CENTER[1] - 2, 60, 4))
        pygame.draw.rect(screen, YELLOW, (CENTER[0] - 45, CENTER[1] - 2, 5, 15))
        # Right Wing
        pygame.draw.rect(screen, YELLOW, (CENTER[0] + 40, CENTER[1] - 2, 60, 4))
        pygame.draw.rect(screen, YELLOW, (CENTER[0] + 40, CENTER[1] - 2, 5, 15))

        # 5. Draw Bezel/Overlay
        pygame.draw.circle(screen, BLACK, CENTER, 180, 10) # Border

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()