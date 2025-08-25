# Introduction to Edge Impulse X Rubik Pi

Edge Impulse is an ML development platform focused on making it easy to build, train, optimize, and deploy ML models directly on edge devices.
For this project, we use Edge Impulse with the Rubik Pi to run quantized models with high accuracy and low inference latency, which is ideal for real-time applications like gesture recognition.

## Developer Workflows

Depending on your experience level and available time, you can choose from three recommended workflows:
- [Beginner Developer](https://github.com/shreyaNagunuri/Edge-Impulse-RubikPi/tree/main/beginner-workflow): No-code experience; get an ML project done quickly with some customization for decent results
- [Average Developer](https://github.com/shreyaNagunuri/Edge-Impulse-RubikPi/tree/main/average-workflow): Low-code experience; add some customization to the neural network architecture for even better results
- [Expert Developer](https://github.com/shreyaNagunuri/Edge-Impulse-RubikPi/tree/main/expert-workflow): Upload your own model into Edge Impulse to utilize the Edge Impulse workflow
  - For the expert developer workflow, I have included a sample training script that can be run to generate a TFLite model.

## Rubik Pi Deployment
To connect the Rubik Pi to Edge Impulse follow the steps found in `enable_edge_impulse.md`. 
To leverage Qualcomm’s Hexagon Delegate on the Rubik Pi:
- Use quantized TFLite models for optimal performance.
- Upload models to Edge Impulse for testing and deployment.
- For direct execution, use the provided `run_model_delegate.py` script to run the Int8 TFLite models locally with accelerated inference.
  
## Sample Project: ASL Classification
To test these workflows, the sample project I will be building is an **ASL Classification Task**
- Combined dataset of three publicly available ASL image datasets: 60k images across 28 classes (A–Z, Nothing, Space)
- Classification model based on MobileNetV2 architecture, leveraging transfer learning


https://github.com/user-attachments/assets/d93ad392-ce46-45d4-b8b5-08016ff99ef0

