# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

import argparse
import tensorflow as tf

def load_model(model_path):
    try:
        # Attempt to load QNN delegate
        try:
            delegate_options = {
                'backend_type': 'htp'
            }
            delegate = tf.lite.experimental.load_delegate('libQnnTFLiteDelegate.so', delegate_options)
            interpreter = tf.lite.Interpreter(
                model_path=model_path,
                experimental_delegates=[delegate]
            )
            print("INFO: Loaded QNN delegate with HTP backend")
        except Exception as e:
            print(f"WARNING: Failed to load QNN delegate: {e}")
            print("INFO: Continuing without QNN delegate")
            interpreter = tf.lite.Interpreter(model_path=model_path)

        interpreter.allocate_tensors()
        print("INFO: Model loaded and tensors allocated successfully")
        return interpreter

    except Exception as e:
        print(f"ERROR: Failed to load model file '{model_path}': {e}")
        return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load a TFLite model with optional QNN delegate.")
    parser.add_argument('--model', type=str, required=True, help='Path to the TFLite model file')
    args = parser.parse_args()

    load_model(args.model)


# python load_model.py --model path/to/your/model.tflite