import os
from sensor_pb2 import Sensor

if __name__ == "__main__":
    with open("sensor.data", 'rb') as file:
        content = file.read()

    print("Retrieve Sensor object from sensor.data")
    sensor = Sensor()
    sensor.ParseFromString(content)
    door = "Open" if sensor.door else "Closed"
    print("Sensor name: {}".format(sensor.name))
    print("Sensor temperature: {}".format(sensor.temperature))
    print("Sensor humidity: {}".format(sensor.humidity))
    print("Sensor door: {}".format(door))
