import math

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# Predicted and measured angles in radians
predicted_angle = math.radians(179)   # ~3.124 radians
measured_angle = math.radians(-179)     # ~ -3.124 radians

raw_error = measured_angle - predicted_angle
normalized_error = normalize_angle(raw_error)

print("Predicted angle (deg):", math.degrees(predicted_angle))
print("Measured angle (deg):", math.degrees(measured_angle))
print("Raw error (deg):", math.degrees(raw_error))
print("Normalized error (deg):", math.degrees(normalized_error))
