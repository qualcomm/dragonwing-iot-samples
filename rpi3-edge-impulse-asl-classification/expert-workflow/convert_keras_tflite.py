# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

import tensorflow as tf

keras_model = tf.keras.models.load_model("model.keras")

converter = tf.lite.TFLiteConverter.from_keras_model(keras_model) 
tflite_model = converter.convert()

with open('tflite_model.tflite', 'wb') as f:     
  f.write(tflite_model)

print("Saved Float Tflite")

converter.optimizations = [tf.lite.Optimize.DEFAULT] 
tflite_model_quantized = converter.convert()

with open('tflite_model_int.tflite', 'wb') as f:     
  f.write(tflite_model)

print("Saved Int Tflite")