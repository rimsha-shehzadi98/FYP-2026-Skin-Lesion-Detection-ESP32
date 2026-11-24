# FYP-2026-Skin-ESP32
# Acne & Skin Disease Detection Using Image Data on ESP32-CAM

## Project Overview
This project is a Final Year Project (FYP) focused on detecting acne and skin diseases using image data captured through an ESP32-CAM module. The system uses a machine learning model trained via Edge Impulse to classify skin conditions in real time on embedded hardware.

## Team
- **Iqra Waheed** — Model Training and Optimization Lead and prototype development
- **Rimsha Shehzadi** — Data Collection, and Hardware Integration
- **Supervisor:** Dr. Usama Ahmad

## How It Works (Project Flow)

1. **Image Capture**
   The ESP32-CAM module captures live images of the skin area.

2. **Preprocessing**
   Captured images are resized and normalized to match the input format required by the trained model.

3. **Model Inference (On-Device)**
   A FOMO MobileNetV2 model, trained on Edge Impulse, runs directly on the ESP32-CAM to classify the image into one of the following classes:
   - Basal Cell Carcinoma (BCC)
   - Nevus
   

4. **Result Display**
   The prediction result, along with the confidence score, is sent to a web-based dashboard hosted by the ESP32-CAM.

5. **Web Dashboard**
   The dashboard displays:
   - Captured image
   - Predicted class and confidence score
   - Confidence trend chart
   - Pie chart of class distribution
   - Confusion matrix
   - Option to export results as CSV

## Model Details
- **Architecture:** FOMO MobileNetV2 (optimized for edge devices)
- **Training Platform:** Edge Impulse
- **Classes:** BCC, Nevus
- **Performance:** ~83.9% F1 Score (BCC and Nevus)

## Hardware Used
- ESP32-CAM Module
- FTDI Programmer (for uploading code)

## Tools & Technologies
- Edge Impulse (Model Training)
- Arduino IDE (Firmware Development)
- MobileNetV2 (Transfer Learning)
- HTML/CSS/JavaScript (Web Dashboard)
