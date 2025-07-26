# 🤖 Assistive Robotic Arm Control Using Eye Tracking

An innovative assistive technology system that enables individuals with mobility impairments to control a 6-DOF robotic arm using **eye movements**. This project combines computer vision, real-time processing, and robotics to provide an intuitive, hands-free control interface.

---

## 🎯 Project Overview

This system empowers users with severe motor impairments to perform daily tasks—such as grasping objects, pointing, and interacting with their environment—using only eye movements. Designed to be **affordable**, **non-invasive**, and **accessible**.

### ✅ Key Features

- 👁️ **Real-time Eye Tracking** – MediaPipe-based accurate gaze detection  
- 🦾 **6-DOF Robotic Arm Control** – Precise servo control using PCA9685  
- 🧠 **Non-invasive Interface** – No wearable or complex setup  
- 🕹️ **Intuitive Control** – Natural eye gestures map to robotic actions  
- ✊ **Grasp Trigger** – Gaze at the center for 7 seconds to toggle gripper  
- 📺 **Visual Feedback** – Live display of status and servo positions  

---
## 🏗️ System Architecture

[User Eye Gaze] → [Camera] → [MediaPipe] → [Raspberry Pi 4] → [PCA9685 Driver] → [6-DOF Robotic Arm]



---

## ⚙️ Hardware Components

| Component | Description |
|----------|-------------|
| **Raspberry Pi 4 (8GB)** | Main processing unit |
| **6-DOF Robotic Arm** | MG996R servo-based manipulator |
| **PCA9685 Driver** | 16-channel PWM controller |
| **USB Webcam** | Eye tracking input |
| **Power Supply** | 5–6V source for servo motors |

---

## 🖥️ Software Stack

- **Python 3.x**
- **MediaPipe** – Eye/face tracking
- **OpenCV** – Vision processing
- **Adafruit PCA9685 & Motor Libraries**
- **Threading** – Smooth concurrent execution

---

## 🚀 Installation & Setup

### 📦 Prerequisites
```bash
sudo apt update && sudo apt upgrade -y
pip install opencv-python mediapipe numpy
pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor

