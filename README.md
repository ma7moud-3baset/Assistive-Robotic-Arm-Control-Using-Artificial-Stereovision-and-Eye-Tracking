# Assistive Robotic Arm Control Using Eye Tracking

An innovative assistive technology system that enables individuals with mobility impairments to control a 6-DOF robotic arm using eye movements. This project combines computer vision, real-time processing, and robotics to provide an intuitive, hands-free control interface.

![System Demo](images/system_demo.gif)
*Real-time eye tracking controlling a 6-DOF robotic arm*

## 🎯 Project Overview

This system empowers users with severe motor impairments to perform daily tasks like grasping objects, pointing, and manipulating their environment using only eye movements. The solution is designed to be affordable, non-invasive, and accessible.

### Key Features
- **Real-time Eye Tracking**: Uses MediaPipe for accurate gaze detection
- **6-DOF Robotic Arm Control**: Precise servo motor control with PCA9685 driver
- **Non-invasive Interface**: No physical contact or complex setup required
- **Intuitive Control**: Natural eye movements translate to robotic actions
- **Grasp Functionality**: Extended center gaze (7 seconds) triggers gripper
- **Visual Feedback**: Real-time display of system status and servo positions

## 🏗️ System Architecture

```mermaid
graph LR
    A[User Eye Gaze] --> B[USB Camera]
    B --> C[MediaPipe Processing]
    C --> D[Raspberry Pi 4]
    D --> E[PCA9685 Driver]
    E --> F[6-DOF Robotic Arm]
    F --> G[Physical Actions]
```

### Hardware Components
- **Raspberry Pi 4 Model B** (8GB RAM) - Main processing unit
- **6-DOF Robotic Arm** - Multi-joint manipulator with MG996R servos
- **PCA9685 16-Channel PWM Driver** - Servo motor control
- **USB Webcam** - Eye tracking input
- **Power Supply** - Stable power for all components

### Software Stack
- **Python 3.x** - Main programming language
- **MediaPipe** - Real-time face mesh and eye tracking
- **OpenCV** - Computer vision processing
- **Adafruit Libraries** - Hardware communication
- **Threading** - Concurrent processing for smooth operation

## 🚀 Installation & Setup

### Prerequisites
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python dependencies
pip install opencv-python
pip install mediapipe
pip install numpy
pip install adafruit-circuitpython-pca9685
pip install adafruit-circuitpython-motor
pip install collections
pip install threading
```

### Hardware Setup

#### 1. Connect Servos to PCA9685
```
Servo Channel Mapping:
- Servo 0 (Base Rotation): Channel 0
- Servo 1 (Shoulder): Channel 1  
- Servo 2 (Elbow): Channel 2
- Servo 3 (Wrist Pitch): Channel 3
- Servo 4 (Wrist Roll): Channel 4
- Servo 5 (Gripper): Channel 5
```

#### 2. Connect PCA9685 to Raspberry Pi
```
PCA9685 Pin    →    Raspberry Pi 4 Pin
VCC            →    5V (Pin 2)
GND            →    GND (Pin 6)  
SDA            →    GPIO 2 (Pin 3)
SCL            →    GPIO 3 (Pin 5)
```

#### 3. Power Connections
- Connect 5-6V external power supply to PCA9685 power terminals
- Ensure sufficient current capacity (minimum 3A recommended)

#### 4. Connect USB Webcam
- Connect to any available USB port on Raspberry Pi

### Software Installation
```bash
# Clone the repository
git clone https://github.com/yourusername/assistive-robotic-arm.git
cd assistive-robotic-arm

# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Enable I2C on Raspberry Pi
sudo raspi-config
# Navigate to Interface Options → I2C → Enable → Finish → Reboot
```

### Create requirements.txt
```txt
opencv-python==4.8.1.78
mediapipe==0.10.7
numpy==1.24.3
adafruit-circuitpython-pca9685==3.4.15
adafruit-circuitpython-motor==3.4.10
adafruit-circuitpython-busdevice==5.2.6
RPi.GPIO==0.7.1
```

## 🎮 Usage

### Running the System
```bash
# Navigate to project directory
cd assistive-robotic-arm

# Activate virtual environment (if using)
source venv/bin/activate

# Run the main application
python paste.py
```

### Control Commands

| Gaze Direction | Action | Servo Changes |
|---------------|--------|---------------|
| **UP** | Move arm upward | Servos 1,2: -15° |
| **DOWN** | Move arm downward | Servos 1,2: +15° |
| **LEFT** | Move arm left | Servo 0: -20° |
| **RIGHT** | Move arm right | Servo 0: +20° |
| **CENTER (7s)** | Toggle gripper | Servo 5: 0°↔180° |

### Keyboard Controls
- `q` - Quit application safely
- `r` - Reset arm to center position

### System Status Display
The real-time interface displays:
- **Control Type**: PCA9685/GPIO/Simulation mode
- **Current Servo Angles**: Live position feedback
- **System Status**: Ready/Calculating/Reading phases
- **Gripper State**: Open/Closed with countdown
- **Last Command**: Most recent executed action
- **Eye Tracking**: Gaze direction and coordinates

## ⚙️ Configuration

### Servo Configuration
```python
# Initial servo positions (degrees)
INITIAL_SERVO_ANGLES = [25, 60, 10, 40, 150, 0]

# Center/reset positions (degrees)  
CENTER_ANGLES = [90, 90, 90, 90, 90, 0]

# Movement increment mappings
MOVEMENT_SCENARIOS = {
    "UP": {1: -15, 2: -15},        # Shoulder & Elbow up
    "DOWN": {1: +15, 2: +15},      # Shoulder & Elbow down  
    "LEFT": {0: -20},              # Base rotation left
    "RIGHT": {0: +20},             # Base rotation right
    "CENTER": {}                   # No movement (gripper control)
}
```

### Timing Configuration
```python
CALCULATION_DELAY = 2    # Command processing delay (seconds)
READING_DELAY = 5        # Additional safety delay (seconds)
TOTAL_DELAY = 7          # Total delay before execution
```

### Eye Tracking Parameters
```python
# MediaPipe Face Mesh settings
min_detection_confidence = 0.5
min_tracking_confidence = 0.5
refine_landmarks = True

# Gaze detection thresholds
horizontal_threshold = 0.25
vertical_threshold = 0.35
```

## 🔧 Troubleshooting

### Common Issues & Solutions

#### Camera Issues
```bash
# Check camera connection
lsusb | grep -i camera

# Test camera functionality
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Error')"

# If camera not found, try different indices
python -c "import cv2; [print(f'Camera {i}: {cv2.VideoCapture(i).isOpened()}') for i in range(4)]"
```

#### I2C Communication Errors
```bash
# Check I2C is enabled
sudo raspi-config nonint get_i2c

# Scan for I2C devices
sudo i2cdetect -y 1
# Should show PCA9685 at address 0x40

# Check I2C permissions
sudo usermod -a -G i2c $USER
# Logout and login again
```

#### Servo Issues
- **Not moving**: Check power supply voltage (5-6V) and current capacity
- **Jittery movement**: Ensure stable power supply, add capacitors if needed
- **Wrong direction**: Verify servo connections to correct PCA9685 channels
- **Limited range**: Check angle limits in code (0-180°)

#### Eye Tracking Problems
- **Poor detection**: Ensure good lighting, camera at eye level
- **Unstable tracking**: Minimize head movement, clean camera lens
- **Wrong gaze direction**: Recalibrate thresholds for your setup

#### Performance Issues
```bash
# Check CPU usage
htop

# Monitor system temperature  
vcgencmd measure_temp

# Free up memory
sudo sync && sudo sysctl -w vm.drop_caches=3
```

## 📊 Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **Frame Rate** | 22-26 FPS | Stable real-time processing |
| **Gaze-to-Actuation Latency** | 350-500ms | Including safety delays |
| **Servo Synchronization** | <80ms | Multi-joint coordination |
| **CPU Utilization** | ~68% | Raspberry Pi 4 (8GB) |
| **System Uptime** | >90 min/session | Continuous operation |
| **Eye Tracking Accuracy** | ±5 pixels | 640x480 resolution |
| **Angular Precision** | ±2.8° | Servo positioning accuracy |

## 🛡️ Safety Features

- **Dual-Phase Delay**: 7-second total delay prevents accidental activation
- **Angle Limiting**: Servo angles constrained to safe ranges (0-180°)
- **Graceful Degradation**: Continues in simulation mode if hardware fails
- **Error Handling**: Comprehensive exception handling for stability
- **Emergency Reset**: 'r' key instantly resets arm to safe position
- **Visual Feedback**: Clear status indicators for user awareness

## 🧪 Testing & Validation

### Accuracy Testing Results
- **Spatial Accuracy**: ±5 pixels on 640x480 resolution
- **Angular Precision**: ±2.8 degrees average variation  
- **Response Time**: 0.35-0.5 seconds average latency
- **Success Rate**: 95% command recognition accuracy

### User Trial Results
**Participants**: 3 users (1 with limited hand mobility, 2 able-bodied)

**Positive Feedback**:
- Intuitive and non-invasive control interface
- Increased sense of autonomy and independence
- Minimal training required for basic operation
- Smooth and predictable arm movements

**Improvement Suggestions**:
- Audio feedback for command confirmation
- Adjustable gaze dwell times for different users
- Better visual indicators for system states

## 🔮 Future Enhancements

### Phase 1: Immediate Improvements
- [ ] **Object Detection**: YOLOv8 integration for intelligent object selection
- [ ] **User Calibration**: Personalized gaze threshold adjustment
- [ ] **Audio Feedback**: Sound confirmation for commands
- [ ] **Configuration GUI**: User-friendly setup interface

### Phase 2: Advanced Features  
- [ ] **Voice Commands**: Backup control modality
- [ ] **Gesture Integration**: Hand gesture support for able users
- [ ] **Learning Algorithm**: Adaptive user behavior learning
- [ ] **Remote Monitoring**: Caregiver dashboard and alerts

### Phase 3: Clinical Integration
- [ ] **Medical Device Certification**: FDA/CE marking compliance
- [ ] **Clinical Trials**: Validation with target user population
- [ ] **Insurance Integration**: Coverage and reimbursement pathways
- [ ] **Rehabilitation Integration**: Therapy program compatibility

### Technical Roadmap
```mermaid
gantt
    title Development Roadmap
    dateFormat  YYYY-MM-DD
    section Phase 1
    Object Detection    :2025-02-01, 60d
    User Calibration    :2025-02-15, 45d
    section Phase 2  
    Voice Commands      :2025-04-01, 90d
    Learning Algorithm  :2025-04-15, 120d
    section Phase 3
    Clinical Trials     :2025-07-01, 180d
    Certification       :2025-08-01, 120d
```

## 📚 Academic Context

This project was developed as part of a Bachelor's degree final year project at the **College of Computing and Information Technology, South Valley University, Egypt** (2025).

### Supervision Team
- **Prof. Ammar Mostafa Hassan** - Project Supervisor
- **Prof. Mohamed AbdElNaser Mohamed** - Co-Supervisor

### Development Team
| Name | Role | Student ID |
|------|------|------------|
| **Ahmed Mohamed Atta Mahmoud** | Team Lead | 211013038 |
| **Mahmoud Abdelbaset Mohamed** | Hardware Integration | 211008597 |
| **Amr Fathy Ahmed Mohamed** | Software Development | 211013265 |
| **Mohamed Ibrahim Abdo** | System Testing | 211008557 |
| **Ahmed Salah Eldin Mahmoud** | Documentation | 211013137 |

### Academic Publications
- Conference paper submitted to IEEE ICRA 2025
- Journal article in preparation for IEEE Transactions on Assistive Technology

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2025 Assistive Robotic Arm Team

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
```

## 🤝 Contributing

We welcome contributions from the community! Please read our [Contributing Guidelines](CONTRIBUTING.md) before submitting.

### How to Contribute
1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/AmazingFeature`)  
3. **Commit** your changes (`git commit -m 'Add some AmazingFeature'`)
4. **Push** to the branch (`git push origin feature/AmazingFeature`)
5. **Open** a Pull Request

### Development Standards
- Follow PEP 8 style guidelines
- Add comprehensive docstrings
- Include unit tests for new features
- Update documentation as needed

### Bug Reports
Please use the [Issue Template](.github/ISSUE_TEMPLATE.md) when reporting bugs.

## 📞 Support & Community

### Getting Help
- 📖 **Documentation**: Check this README and inline code comments
- 🐛 **Bug Reports**: [Open an issue](https://github.com/yourusername/assistive-robotic-arm/issues)
- 💡 **Feature Requests**: [Discussions page](https://github.com/yourusername/assistive-robotic-arm/discussions)
- 📧 **Contact**: [project-email@university.edu](mailto:project-email@university.edu)

### Community
- Join our [Discord Server](https://discord.gg/assistive-robotics)
- Follow updates on [Twitter](https://twitter.com/assistive_arm)
- Star ⭐ the repository if you find it useful!

## 🙏 Acknowledgments

### Technology Partners
- **Google Research** - MediaPipe framework
- **Adafruit Industries** - Hardware libraries and documentation
- **OpenCV Foundation** - Computer vision tools
- **Raspberry Pi Foundation** - Single-board computer platform

### Inspiration & Research
- **Assistive Technology Research Community** - Guidance and best practices
- **IEEE RAS Technical Committee on Assistive & Rehabilitation Robotics**
- **International Association of Gerontechnology**

### Special Thanks
- **Arab Academy for Science, Technology and Maritime Transport** - Primary sponsor
- **South Valley University** - Research facilities and support
- **Study participants** - Valuable feedback and testing
- **Open source community** - Tools and libraries that made this possible

## 📈 Impact & Applications

### Target Demographics
- **Primary**: Individuals with quadriplegia or severe motor impairments
- **Secondary**: Stroke patients with limited mobility
- **Tertiary**: Elderly users requiring assistance with daily tasks

### Use Cases
- **Activities of Daily Living (ADLs)**:
  - Eating and drinking assistance
  - Object manipulation and retrieval
  - Personal care tasks
  
- **Communication & Interaction**:
  - Pointing and gesturing
  - Environmental control
  - Emergency signaling

- **Rehabilitation & Therapy**:
  - Motor skill assessment
  - Cognitive-motor training
  - Progress tracking

### Social Impact
- **Independence**: Reduced reliance on caregivers
- **Dignity**: Maintained personal autonomy
- **Quality of Life**: Enhanced daily living capabilities
- **Economic**: Potential healthcare cost reduction

### Market Potential
- **Global assistive technology market**: $26.8 billion (2022)
- **Projected growth**: 7.4% CAGR through 2030
- **Target price point**: <$2,000 (vs $15,000+ for commercial alternatives)

---

## 📱 Quick Start Guide

### For Researchers
```bash
git clone https://github.com/yourusername/assistive-robotic-arm.git
cd assistive-robotic-arm
pip install -r requirements.txt
python paste.py
```

### For Developers
```bash
# Development setup
git clone https://github.com/yourusername/assistive-robotic-arm.git
cd assistive-robotic-arm
python -m venv dev-env
source dev-env/bin/activate
pip install -r requirements-dev.txt
pre-commit install
```

### For End Users
1. Download the latest [release](https://github.com/yourusername/assistive-robotic-arm/releases)
2. Follow the [Installation Guide](#-installation--setup)
3. Run the [Setup Wizard](docs/setup-wizard.md)
4. Complete the [User Tutorial](docs/user-tutorial.md)

---

**🤖 This system represents a step toward more accessible and affordable assistive robotics. Together, we can build technology that empowers everyone to live more independently.**

---

*Last updated: January 2025 | Version 1.0.0*