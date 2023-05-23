#include <iostream>
#include <fstream>
#include "sensor.pb.h"

int main() {
    Sensor sensor;
    sensor.set_name("Laboratory");
    sensor.set_temperature(23.4);
    sensor.set_humidity(68);
    sensor.set_door(Sensor_SwitchLevel_OPEN);

    std::cout << "Serialize " << sensor.name() << " to sensor.data\n";
    std::ofstream ofs("sensor.data");
    sensor.SerializeToOstream(&ofs);

    return 0;
}


