# DESIGN-THINKING-4TH-SEM
Active 3 Axis Stabilization System for UAV Cargos
# Summary
# Work:
Designed a gimbal system to isolate a UAV cargo platform from external disturbances by translating mapped PWM commands into mechanical counter rotations across the yaw, pitch and roll axis.
# Results:
Working prototype that uses an ESP32 and MPU6050 with a closed loop architecture that converts quaternion-derived euler angles into independent 50 Hz PWM signals across 3 GPIO channels for 3 servos, driving synchronized counter-rotation servos to hold the cargo platform level against physical disturbance.
