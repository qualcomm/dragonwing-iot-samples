# Expert Workflow - Edge Impulse

This README outlines an expert developer workflow for building and deploying an American Sign Language (ASL) classification model using TensorFlow and Edge Impulse.

Unlike the beginner workflow, where developers rely on Edge Impulse’s default settings, this approach gives you full control over model architecture, training, and optimization before uploading to Edge Impulse for deployment and testing.

Objective: Train a MobileNetV2-based image classification model for ASL gestures using TensorFlow, export it to TFLite format, and deploy it to Edge Impulse for real-time testing and optimization.

## 1. Install Dependencies
```bash
pip install tensorflow numpy
```

## 2. Run Training Script
```bash
python train_expert_model.py \
  --data_dir data/Train \
  --weights_path expert-workflow/mobilenet_v2_weights_tf_dim_ordering_tf_kernels_1.0_160_no_top.h5 \
  --output_dir expert-workflow/outputs \
  --epochs 10 \
  --batch_size 128
```

## 3. Export to TFLite
Run the `convert_keras_tflite.py` script to automatically save 
- `model_float.tflite`
- `model_int8.tflite`

## 4. Upload Model to Edge Impulse
To upload your model, create a project and select the **Dashboard** tab. Next, click to **Upload your model**. Just as a warning, switching will clear your existing impulse and all your configured blocks, but your data will not be removed. Follow the steps to submit your model (SavedModel, ONNX, TFLite, LGBM, XGBoost, or pickle model). 

## 5. Model testing / Deployment
Go to the **Model Testing** tab and click **Classify all** in order to test the data with your new Impulse. To deploy, select the Deployment tab and either select to "Launch in browser" or run on the Rubik Pi using:

``` bash
edge-impulse-linux-runner
```


