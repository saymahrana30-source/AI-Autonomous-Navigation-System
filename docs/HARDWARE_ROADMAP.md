# From simulation to a real robot (optional)

The simulator is built so the *brain* (`perception → planner → controller`) never touches the world directly - it only sees `Scan` objects. To move to hardware you replace the simulator parts, not the brain:

| Simulated part | Real replacement |
|---|---|
| `Lidar.scan()` | RPLidar / HC-SR04 sweep on a servo, returning the same `Scan(ranges, hit, rel_angles)` |
| `Simulation._actuate` | motor driver (L298N) PWM from `(v, w)` |
| `agent.pose` | wheel odometry / IMU (add a localisation step) |

## Problems in the hardware section of the original brief (fix before wiring anything)
1. **GPIO 23 is used twice** - ultrasonic TRIG and motor-driver IN4. Move one of them (e.g. IN4 → GPIO 25).
2. **HC-SR04 ECHO outputs 5 V** but Raspberry Pi GPIO is 3.3 V tolerant only. Use a voltage divider (1 kΩ + 2 kΩ) on ECHO or the Pi pin can be damaged.
3. **L298N VCC is the motor supply**, not a 5 V logic pin from the Pi - power motors from the battery pack, share GND with the Pi.
4. **The YOLO snippet only reads and shows frames** - it never calls the network. A real detector needs `net.setInput(...)`, `net.forward(...)` and non-max suppression. (Use `ultralytics` YOLOv8n for a quick, working version.)
5. The ultrasonic loop has no timeout - add one or it hangs if no echo returns.
