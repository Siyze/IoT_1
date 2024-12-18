from imu import MPU6050
import time
from machine import Pin, I2C
import math

class CalorieCalculator:
    def __init__(self, weight_kg=92):  
        self.weight_kg = weight_kg
        self.last_time = time.time()
        self.total_calories = 0
        self.met_value = 1.0  # Metabolisk ækvivalent (1 MET = hvilestofskifte)
    
    def calculate_movement_intensity(self, accel, gyro):
        # Beregn samlet bevægelsesintensitet fra accelerometer og gyroskop
        accel_magnitude = math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)
        gyro_magnitude = math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2)
        
        # Juster MET-værdien baseret på bevægelsesintensitet
        if accel_magnitude < 1.2 and gyro_magnitude < 10:
            self.met_value = 1.0  # Hvile
        elif accel_magnitude < 2.0 and gyro_magnitude < 30:
            self.met_value = 2.0  # Let aktivitet
        elif accel_magnitude < 3.0 and gyro_magnitude < 50:
            self.met_value = 4.0  # Moderat aktivitet
        else:
            self.met_value = 6.0  # Intens aktivitet
    
    def calculate_calories(self):
        current_time = time.time()
        time_diff = current_time - self.last_time
        
        # Kalorieforbrænding = MET * 3.5 * vægt(kg) * tid(min) / 200
        calories_burned = (self.met_value * 3.5 * self.weight_kg * (time_diff / 60)) / 200
        
        self.total_calories += calories_burned
        self.last_time = current_time
        return calories_burned

# Initialisering
i2c = I2C(0)
imu = MPU6050(i2c)
calorie_calculator = CalorieCalculator(weight_kg=70)  # Angiv din vægt her

print("Starter kaloriemåling...")
print("Temperatur: ", round(imu.temperature,2), "°C")

while True:
    # Læs sensordata
    acceleration = imu.accel
    gyroscope = imu.gyro
    
    # Beregn kalorier
    calorie_calculator.calculate_movement_intensity(acceleration, gyroscope)
    calories_burned = calorie_calculator.calculate_calories()
    
    # Vis data
    print("\nAcceleration x:", round(acceleration.x,2), "y:", round(acceleration.y,2),
          "z:", round(acceleration.z,2))
    print("Gyroscope x:", round(gyroscope.x,2), "y:", round(gyroscope.y,2),
          "z:", round(gyroscope.z,2))
    print(f"Aktuel MET: {calorie_calculator.met_value}")
    print(f"Forbrændte kalorier (total): {round(calorie_calculator.total_calories, 2)} kcal")
    
    time.sleep(5.0)  # Opdater hver sekund