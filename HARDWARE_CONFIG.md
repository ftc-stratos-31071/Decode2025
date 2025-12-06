# FTC Robot Hardware Configuration

**Last Updated:** December 6, 2025  
**Configuration Name:** `FTC_Robot_Config`

---

## Control Hub

### Motors
| Port | Device Name in Code | Physical Device | Direction | Brake Mode |
|------|---------------------|-----------------|-----------|------------|
| 0 | `IntakeMotor` | Intake roller motor | Reversed | Brake |
| 1 | `TurretMotor` | Turret rotation motor | Reversed | Float |
| 2 | `frontLeftMotor` | Front left mecanum drive | Reversed | Brake |
| 3 | `backLeftMotor` | Back left mecanum drive | Reversed | Brake |

### I2C Sensors
| Port | Device Name | Type |
|------|-------------|------|
| 0 | `limelight` | Limelight3A Vision Camera |

---

## Expansion Hub

### Motors
| Port | Device Name in Code | Physical Device | Direction | Brake Mode |
|------|---------------------|-----------------|-----------|------------|
| 0 | `ShooterRight` | Right shooter flywheel | Normal | Brake |
| 1 | `ShooterLeft` | Left shooter flywheel | Reversed | Brake |
| 2 | `backRightMotor` | Back right mecanum drive | Normal | Brake |
| 3 | `frontRightMotor` | Front right mecanum drive | Normal | Brake |

### Servos
| Port | Device Name | Description | Type |
|------|-------------|-------------|------|
| 0 | `IntakeServo` | Intake folding/positioning servo | Standard |
| 1 | `HoodServo` | Shooter hood angle servo | Standard |

---

## Hardware Configuration Steps

### In FTC Robot Controller App:

1. **Configure Control Hub:**
   - Motor Port 0: "IntakeMotor" → REV Robotics Core Hex Motor (direction: reverse)
   - Motor Port 1: "TurretMotor" → REV Robotics Core Hex Motor (direction: reverse)
   - Motor Port 2: "frontLeftMotor" → REV Robotics Core Hex Motor (direction: reverse)
   - Motor Port 3: "backLeftMotor" → REV Robotics Core Hex Motor (direction: reverse)
   - I2C Port 0: "limelight" → Limelight3A

2. **Configure Expansion Hub:**
   - Motor Port 0: "ShooterRight" → REV Robotics Core Hex Motor (direction: forward)
   - Motor Port 1: "ShooterLeft" → REV Robotics Core Hex Motor (direction: reverse)
   - Motor Port 2: "backRightMotor" → REV Robotics Core Hex Motor (direction: forward)
   - Motor Port 3: "frontRightMotor" → REV Robotics Core Hex Motor (direction: forward)
   - Servo Port 0: "IntakeServo" → Servo
   - Servo Port 1: "HoodServo" → Servo

3. **Save Configuration** as "FTC_Robot_Config"

---

## Important Notes

⚠️ **Motor Names Must Match Exactly** - The device names in the Robot Controller configuration MUST match the names in the code exactly (case-sensitive):
- `frontLeftMotor`, `backLeftMotor`, `frontRightMotor`, `backRightMotor` (drive)
- `IntakeMotor` (intake)
- `TurretMotor` (turret)
- `ShooterRight`, `ShooterLeft` (shooter)
- `IntakeServo`, `HoodServo` (servos)
- `limelight` (camera)

---

## Wiring Notes

### Drive Train (Mecanum)
- **Control Hub Ports 2 & 3:** Left side drive motors (frontLeftMotor, backLeftMotor)
- **Expansion Hub Ports 2 & 3:** Right side drive motors (backRightMotor, frontRightMotor)
- All drive motors use brake mode for precise stopping
- Left motors are reversed in code

### Shooter System
- **Both shooter motors** on Expansion Hub (ports 0 & 1)
- ShooterLeft motor is reversed to spin in opposite direction
- Hood servo controls shooter angle (Expansion Hub servo port 1)

### Intake System
- **Intake motor** on Control Hub port 0
- **Intake servo** on Expansion Hub servo port 0
- Motor is reversed for correct intake direction

### Turret System
- **Turret motor** on Control Hub port 1
- Motor is reversed for correct rotation direction
- Uses PID control for smooth tracking

### Vision System
- **Limelight3A** connected to Control Hub I2C port 0
- Provides AprilTag detection for auto-tracking
- Streams video to FTC Dashboard

---

## Quick Reference

### Subsystem Mappings

**Intake (Intake.java):**
- Motor: `IntakeMotor` (Control Hub port 0)
- Servo: `IntakeServo` (Expansion Hub servo port 0)

**Shooter (Shooter.java):**
- Motor 1: `ShooterRight` (Expansion Hub motor port 0)
- Motor 2: `ShooterLeft` (Expansion Hub motor port 1)
- Servo: `HoodServo` (Expansion Hub servo port 1)

**Turret (Turret.java):**
- Motor: `TurretMotor` (Control Hub port 1)

**Drive Train (Teleop.java):**
- Front Left: `frontLeftMotor` (Control Hub port 2)
- Back Left: `backLeftMotor` (Control Hub port 3)
- Front Right: `frontRightMotor` (Expansion Hub port 3)
- Back Right: `backRightMotor` (Expansion Hub port 2)

---

## Troubleshooting

### Motor Not Responding
1. Check physical connection to correct hub and port
2. Verify device name matches exactly (case-sensitive)
3. Check motor direction setting in code vs. configuration

### Strafing Not Working
1. Verify all 4 drive motors are connected and configured correctly
2. Check that motor names match exactly in configuration
3. Test each motor individually to ensure they're working
4. Verify left motors are on Control Hub, right motors on Expansion Hub

### Servo Jittering
1. Ensure servo has good power connection
2. Check servo position values are between 0.0 and 1.0
3. Verify servo is on Expansion Hub (better power distribution)

### Limelight Not Detected
1. Check I2C connection to Control Hub port 0
2. Verify "limelight" device name in configuration
3. Ensure Limelight is powered on (green LED)
4. Check network connection to Limelight

---

## Configuration Summary

All motor/servo names in code remain unchanged. Only the physical port locations are documented here:

- ✅ **Motor names in code:** frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, IntakeMotor, TurretMotor, ShooterRight, ShooterLeft
- ✅ **Servo names in code:** IntakeServo, HoodServo
- ✅ **Code files:** No changes needed - all hardware names were already correct!
