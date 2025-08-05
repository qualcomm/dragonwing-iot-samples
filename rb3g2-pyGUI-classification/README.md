# 🧠 Image Classification

This sample Python application demonstrates how to deploy and run a classification model on Qualcomm’s RB3 Gen2 and Rubik Pi development platforms. These platforms are designed to accelerate computing and connectivity for consumer and industrial IoT (Internet of Things) devices.

The application is compatible with both Ubuntu 22.04 and Qualcomm Linux 1.4. It provides a graphical interface that allows users to:
Select an image
Run inference using a classification model
View the top 4 predictions with confidence bars
The model used is a quantized GoogLeNet implemented with LiteRT, formerly known as TensorFlow Lite—Google’s high-performance runtime for on-device AI.

---

# 📱 Hardware

   ✅ RB3 Gen 2
   ✅ Rubik Pi

---

## 🖥️ Requirements

### RB3 Gen2 Device
Follow RB3 Gen2 vision kit set up [instructions](https://docs.qualcomm.com/bundle/publicresource/topics/80-70020-251/set_up_the_device.html?product=1601111740013077&facet=User%20Guide) 

### RB3 Gen2 Device
Follow Rubik Pi set up [instructions](https://www.thundercomm.com/rubik-pi-3/en/docs/about-rubikpi/)

### Classification Model and Labels
Download:
- TFLite GoogLeNet-Quantized model from [AIHub](https://aihub.qualcomm.com/models/googlenet?domain=Computer+Vision&useCase=Image+Classification)
- Imagenet dataset [labels](https://github.com/quic/ai-hub-models/blob/main/qai_hub_models/labels/imagenet_labels.txt)

---

## 🎮 Platform OS Setup:

### Ubuntu:
Clone the project directly on your RB3 Gen2
   ```bash
   git clone https://github.com/cyberZil/rb3g2-pyGUI-classification.git
   cd rb3g2-pyGUI-classification
   pip install -r requirements.txt
   ```

### Qualcomm Linux:
Clone repository on your host machine and push files to RB3 Gen 2:
   ```bash
   adb push rb3g2-pyGUI-classification /opt
   ```

---

## 📁 File Structure on Device
```
main.py                    # Main application script
googlenet_quantized.tflite # TFLite model file (required)
imagenet_labels.txt        # Label file (required)
MainWindowPic.jpg          # Default image
requirements.txt           # Libraries needed to run app
```

---

## 🚀 How to Run App on RB3 Gen 2
After going to the folder where you cloned the repo or /opt for Qualcomm Linux run the following command
   ```bash
   python3 main.py
   ```
Output
 ![image](https://github.com/user-attachments/assets/d927546b-459b-4332-816a-9b8d505679a6)
---

#📺 Use GUI and expected output

  - Click **"Select Image"** to choose an image file.
    ![image](https://github.com/user-attachments/assets/4f61215b-dbd2-4ee7-b185-650f360665c2)
  - The image will be displayed and classified with the top 4 predictions.
    ![image](https://github.com/user-attachments/assets/009beb3c-bbb1-40a9-b97b-18a8928bb95c)

---

## 🛠️ Customization

- To use a different model, update the `TF_MODEL` and `LABELS` constants in the script.
- You can replace `MainWindowPic.jpg` with your own default image.
- Modify the `runInference()` function to support additional output tensors or model types.

---
