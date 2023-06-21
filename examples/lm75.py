from regmap.core.models import I2cBus
from regmap.devices.temp import LM75


bus = I2cBus()
sensor = LM75(bus)

transactions = [
    sensor.configuration.write(0b00000100),  # OS active high
    sensor.thyst.write(75 << 8),  # OS inactive when temp falls under 75°C
    sensor.tos.write(80 << 8),  # OS active when temp rises above 80°C
    sensor.temperature.read(),
]

for t in transactions:
    print(f"{t}")
