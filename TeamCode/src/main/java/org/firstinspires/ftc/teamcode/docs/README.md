# TeamCode Documentation

Welcome to the FTC robot code documentation! This directory contains all guides for setting up, tuning, and operating the robot systems.

---

## 📚 Quick Navigation

### 🎯 Vision & Aiming
- **[Camera Alignment Guide](CAMERA_ALIGNMENT_GUIDE.md)** - AprilTag tracking and turret auto-aiming

### 🚀 Shooter System
- **[Shooter System Overview](SHOOTER_SYSTEM_OVERVIEW.md)** - How the PID velocity control works
- **[Shooter PID Tuning Guide](SHOOTER_PID_TUNING_GUIDE.md)** - Step-by-step tuning with FTC Dashboard

### 🤖 Autonomous Navigation
- **[Pedro Pathing Setup Guide](PEDRO_SETUP_TUNING_GUIDE.md)** - Complete Pedro setup and tuning
- **[Pedro Setup Guide (Quick)](PEDRO_SETUP_GUIDE.md)** - Quick reference version

---

## 🔧 What's New

### Camera Alignment Fix (Latest)
The turret now uses **PID control** instead of simple scaling for AprilTag alignment:

**Before:**


**After:**


**Benefits:**
- ✅ Perfectly centers on target
- ✅ No oscillation or jitter
- ✅ Smooth tracking of moving targets
- ✅ Live tunable via FTC Dashboard

See [Camera Alignment Guide](CAMERA_ALIGNMENT_GUIDE.md) for full details.

---

## 🎮 System Overview

### Subsystems
1. **Drivetrain** - Mecanum drive with field-centric control
2. **Shooter** - Dual-motor flywheel with PID velocity control
3. **Turret** - Vision-guided aiming with PID position control
4. **Intake** - Ball collection and feeding system

### Key Features
- 🎯 **Auto-Aiming** - Limelight vision + turret tracking
- 🔋 **Battery Compensation** - Consistent shooter RPM throughout match
- 🛣️ **Pedro Pathing** - Advanced autonomous path following
- 📊 **FTC Dashboard** - Live tuning of all PID parameters

---

## 🚀 Quick Start

### For New Team Members

1. **Start here:** [Shooter System Overview](SHOOTER_SYSTEM_OVERVIEW.md)
   - Understand how the shooter works
   - Learn about PID control and why it matters

2. **Then read:** [Camera Alignment Guide](CAMERA_ALIGNMENT_GUIDE.md)
   - Understand vision tracking
   - Learn about turret control

3. **For autonomous:** [Pedro Pathing Setup Guide](PEDRO_SETUP_TUNING_GUIDE.md)
   - Complete setup from scratch
   - Tune all parameters

### For Tuning

All systems can be tuned live via **FTC Dashboard** at `http://192.168.43.1:8080/dash`

**Tunable Parameters:**
- Shooter: `ShooterConstants` (RPM, PID gains)
- Turret: `TurretConstants` (Position PID)
- Alignment: `Teleop` (Vision tracking PID)
- Pedro: `PedroConstants` (Path following PID)

No code changes needed - just adjust values in dashboard!

---

## 📖 Guide Details

### Camera Alignment Guide
**What it covers:**
- How the vision system works
- PID control for turret tracking
- Tuning parameters (Kp, Ki, Kd, deadband)
- Testing and troubleshooting
- Integration with autonomous

**When to use:**
- Turret not centering on AprilTag
- Oscillation or jitter issues
- Want to understand vision tracking

---

### Shooter System Overview
**What it covers:**
- System architecture
- PID velocity control explained
- RPM calculation
- Battery compensation
- Usage examples

**When to use:**
- Understanding how shooter works
- Troubleshooting inconsistent shots
- Learning about feedforward + feedback control

---

### Shooter PID Tuning Guide
**What it covers:**
- Step-by-step tuning process
- Using FTC Dashboard for live tuning
- Battery drain testing
- Performance validation

**When to use:**
- Setting up shooter for first time
- Shots inconsistent as battery drains
- Need to change target RPM

---

### Pedro Pathing Setup Guide
**What it covers:**
- Complete setup from scratch
- Localizer configuration (Pinpoint)
- Automatic tuners (velocity, acceleration)
- Manual PIDF tuning (translational, heading, drive)
- Testing and validation

**When to use:**
- Setting up autonomous for first time
- Robot doesn't follow paths accurately
- Need to retune after mechanical changes

---

## 🔍 Troubleshooting

### Common Issues

**Turret not aligning with AprilTag:**
→ See [Camera Alignment Guide](CAMERA_ALIGNMENT_GUIDE.md) - Section: "Common Issues & Solutions"

**Shooter RPM inconsistent:**
→ See [Shooter PID Tuning Guide](SHOOTER_PID_TUNING_GUIDE.md) - Section: "Battery Drain Test"

**Robot overshoots paths:**
→ See [Pedro Pathing Setup Guide](PEDRO_SETUP_TUNING_GUIDE.md) - Section: "Drive PIDF Tuning"

**General errors:**
→ Check FTC Dashboard telemetry for specific error messages

---

## 💡 Tips & Best Practices

### Competition Day
1. **Charge batteries fully** - PID compensates, but start with good voltage
2. **Test alignment first** - Point at AprilTag, verify telemetry shows aligned
3. **Warm up shooter** - Run for 10 seconds to verify RPM stability
4. **Check limits** - Ensure turret doesn't hit mechanical stops

### Practice & Tuning
1. **Tune one system at a time** - Don't change everything at once
2. **Document values** - Write down working values in case you need to revert
3. **Use telemetry** - Watch the numbers, not just the robot behavior
4. **Test edge cases** - Low battery, extreme positions, rapid movements

### Code Changes
1. **Test in practice mode first** - Never deploy untested code to competition
2. **Keep backups** - Git commit before major changes
3. **Update documentation** - If you change how something works, update the guide

---

## 📊 System Performance

### Expected Performance Metrics

**Shooter:**
- Spin-up time: 0.7-1.2 seconds
- RPM consistency: ±20 RPM (vs ±200 RPM without PID)
- Battery compensation: Maintains target RPM from 13V to 11V

**Alignment:**
- Acquisition time: 0.3-0.8 seconds
- Tracking accuracy: ±1° of center
- Stability: No jitter when locked

**Autonomous:**
- Path following: ±1 inch accuracy
- Heading hold: ±2° accuracy
- Repeatability: Consistent across multiple runs

---

## 🛠️ Development Notes

### Architecture
- **NextFTC Framework** - Command-based structure
- **Subsystem Pattern** - Each mechanism is isolated
- **PID Control** - Feedback loops for all motion
- **Live Tuning** - FTC Dashboard integration

### Dependencies
- NextFTC Core & Hardware
- Pedro Pathing
- FTCLib (utilities)
- Limelight Vision
- FTC Dashboard

---

## 📝 Contributing

When adding new features or fixes:

1. **Test thoroughly** - Verify in multiple scenarios
2. **Document changes** - Update relevant guide files
3. **Add telemetry** - Make debugging easier for future
4. **Tune via Dashboard** - Use `@Config` for live tuning

---

## 📞 Support

For questions or issues:

1. Check the relevant guide in this directory
2. Review telemetry output for error messages
3. Test in isolation (disable other subsystems)
4. Ask experienced team members

---

## 🎯 Competition Checklist

Before each match:

- [ ] Batteries fully charged
- [ ] Limelight detecting AprilTags
- [ ] Turret aligns when pointed at target
- [ ] Shooter spins to target RPM
- [ ] Autonomous paths loaded and tested
- [ ] All limits (turret, shooter) configured correctly
- [ ] Telemetry showing expected values
- [ ] Gamepad buttons mapped correctly

**Good luck at competition!** 🏆

---

*Last updated: November 28, 2025*

