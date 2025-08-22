# Average Workflow - Edge Impulse

I define the average developer workflow on Edge Impulse to be a developer who **modifies the default training code** to improve reproducibility and add some customization.  

Here are the steps for this average workflow to accomplish the ASL Classification task.  

## 1. Upload the training/testing images into Edge Impulse
- Same as the beginner workflow: upload your ASL dataset into Edge Impulse (training + testing sets).  

## 2. Create the Impulse
- Add in the image data block.  
- Use **resize mode = 96x96** for MobileNetV2.  
- Add a Transfer Learning block with MobileNetV2.  

## 3. Image
- In the Image tab, click **Save parameters** and **Generate features**.  
- This step extracts image features and usually takes ~15 minutes depending on dataset size.  

## 4. Transfer Learning with Deterministic Training
Instead of using the default Edge Impulse training pipeline, switch to the **Expert Mode Training Script** and paste in the custom code (see below).  

This code includes:
- Augmentations (random flip, crop, brightness).  
- Callbacks:
  - `ModelCheckpoint` for saving best models.
  - `EarlyStopping` for preventing overfitting.
  - `BatchLoggerCallback` with determinism awareness.  
- Fine-tuning phase: unfreezing top layers after initial training for higher accuracy.

To find more ways to modify the code to improve accuracy and efficiency, check out this documentation by Edge Impulse: [Neural Networks documentation](https://docs.edgeimpulse.com/knowledge/concepts/machine-learning/neural-networks)

The settings included in the code are those that I found to work best for this dataset.

### Running the training
1. Open **Transfer Learning → Switch to Keras (Expert) mode**.  
2. Paste in the code from `neural-network-arch.py` (provided above).  
3. Train the model with:  

## 5. Model Testing / Deployment
Go to the **Model Testing** tab and click **Classify all** to run an evaluation on your test set. Check metrics such as accuracy, confusion matrix, and per-class F1 scores.

For deployment: 
Browser demo: Deployment → Launch in browser

Rubik Pi or Linux device: 
```bash
edge-impulse-linux-runner
```
