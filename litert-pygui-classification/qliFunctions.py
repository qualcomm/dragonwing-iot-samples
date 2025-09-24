# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

# qliFunctions.py
import os

DEVICE_OS="Ubuntu"
UNAME = os.uname().nodename

def checkOS():
    
    # Define constants based on Device OS --> move to new file
    if UNAME == "qcs6490-rb3gen2-vision-kit":
        import ctypes

        DEVICE_OS="QualcommLinux"
        tflite = ctypes.CDLL('libtensorflowlite_c.so')
        ctypes.CDLL("libQnnTFLiteDelegate.so")
        
        class TfLiteExternalDelegateOptions(ctypes.Structure):
            _fields_ = [("lib_path", ctypes.c_char_p),
                        ("count", ctypes.c_int),
                        ("keys", ctypes.c_char_p * 256),
                        ("values", ctypes.c_char_p * 256),
                        ("insert", ctypes.c_void_p),]

        # ========= TFLite Function Signatures =========
        tflite.TfLiteModelCreateFromFile.restype = ctypes.c_void_p
        tflite.TfLiteModelCreateFromFile.argtypes = [ctypes.c_char_p]
        tflite.TfLiteInterpreterOptionsCreate.restype = ctypes.c_void_p
        tflite.TfLiteInterpreterCreate.restype = ctypes.c_void_p
        tflite.TfLiteInterpreterCreate.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        tflite.TfLiteInterpreterAllocateTensors.argtypes = [ctypes.c_void_p]
        tflite.TfLiteInterpreterGetInputTensor.restype = ctypes.c_void_p
        tflite.TfLiteInterpreterGetInputTensor.argtypes = [ctypes.c_void_p, ctypes.c_int]
        tflite.TfLiteTensorNumDims.restype = ctypes.c_int
        tflite.TfLiteTensorNumDims.argtypes = [ctypes.c_void_p]
        tflite.TfLiteTensorDim.restype = ctypes.c_int
        tflite.TfLiteTensorDim.argtypes = [ctypes.c_void_p, ctypes.c_int]
        tflite.TfLiteTensorType.restype = ctypes.c_int
        tflite.TfLiteTensorType.argtypes = [ctypes.c_void_p]
        tflite.TfLiteTensorCopyFromBuffer.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
        tflite.TfLiteTensorCopyToBuffer.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
        tflite.TfLiteInterpreterInvoke.argtypes = [ctypes.c_void_p]
        tflite.TfLiteInterpreterGetOutputTensorCount.restype = ctypes.c_int
        tflite.TfLiteInterpreterGetOutputTensorCount.argtypes = [ctypes.c_void_p]
        tflite.TfLiteInterpreterGetOutputTensor.restype = ctypes.c_void_p
        tflite.TfLiteInterpreterGetOutputTensor.argtypes = [ctypes.c_void_p, ctypes.c_int]
        tflite.TfLiteInterpreterDelete.argtypes = [ctypes.c_void_p]
        tflite.TfLiteModelDelete.argtypes = [ctypes.c_void_p]
        tflite.TfLiteInterpreterOptionsDelete.argtypes = [ctypes.c_void_p]
        tflite.TfLiteInterpreterOptionsSetUseNNAPI.restype = ctypes.c_void_p
        tflite.TfLiteInterpreterOptionsSetUseNNAPI.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        
        # ========= TFLite Delegate Function Signatures =========
        tflite.TfLiteDelegateCreate.restype = ctypes.c_void_p
        tflite.TfLiteDelegateCreate.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        tflite.TfLiteInterpreterModifyGraphWithDelegate.restype = ctypes.c_void_p
        tflite.TfLiteInterpreterModifyGraphWithDelegate.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        tflite.TfLiteExternalDelegateOptionsInsert.restype = ctypes.c_int # TfLiteStatus
        tflite.TfLiteExternalDelegateOptionsInsert.argtypes = [ctypes.POINTER(TfLiteExternalDelegateOptions), ctypes.c_char_p, ctypes.c_char_p]
        tflite.TfLiteExternalDelegateOptionsDefault.restype = TfLiteExternalDelegateOptions
        tflite.TfLiteExternalDelegateOptionsDefault.argtypes = [ctypes.c_void_p]
        tflite.TfLiteExternalDelegateCreate.argtypes = [ctypes.POINTER(TfLiteExternalDelegateOptions)]
        tflite.TfLiteExternalDelegateCreate.restype = ctypes.c_void_p
        tflite.TfLiteInterpreterOptionsAddDelegate.restype = None
        tflite.TfLiteInterpreterOptionsAddDelegate.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        tflite.TfLiteExternalDelegateDelete.argtypes = [ctypes.c_void_p]
        tflite.TfLiteExternalDelegateDelete.restype = None

    else:
        import ai_edge_litert.interpreter as tflite
        DEVICE_OS = "Ubuntu"

    print("Checking OS")
    return DEVICE_OS

def inferenceQLI(image, use_delegate, TF_MODEL, DELEGATE_PATH, LABELS):
    results = [] 
    print(f"Running on {DEVICE_OS} using Delegate: {use_delegate}")
        # Load the TFLite model
    model = tflite.TfLiteModelCreateFromFile(TF_MODEL.encode("utf-8"))
    if not model:
        raise RuntimeError("Failed to load model.")
    interpreter = None
    options = tflite.TfLiteInterpreterOptionsCreate()
    
    if use_delegate:
        try:
            print("Attempting to load QNN delegate")
            delegate_options = tflite.TfLiteExternalDelegateOptionsDefault(DELEGATE_PATH.encode("utf-8")) 
            if not delegate_options:
                raise RuntimeError("Failed to create delegate options")
            
            # Insert key-value option
            status = tflite.TfLiteExternalDelegateOptionsInsert(ctypes.byref(delegate_options), b"backend_type", b"htp")
            if status != 0:
                raise RuntimeError("Failed to insert delegate option")

            delegate = tflite.TfLiteExternalDelegateCreate(ctypes.byref(delegate_options))
            if not delegate:
                raise RuntimeError("Delegate creation failed")
        
            tflite.TfLiteInterpreterOptionsAddDelegate(options, delegate)

        except Exception as e:
            print(f"WARNING: Failed to load QNN delegate: {e}")
            print("INFO: Continuing without QNN delegate")

    else:
        
        tflite.TfLiteInterpreterOptionsSetUseNNAPI(options, True)    
                
    interpreter = tflite.TfLiteInterpreterCreate(model, options)
    if interpreter is None:
        raise RuntimeError("Failed to create interpreter")
    
    tflite.TfLiteInterpreterAllocateTensors(interpreter)
    # Get and Prepare input
    input_tensor = tflite.TfLiteInterpreterGetInputTensor(interpreter, 0)
    input_dims = tflite.TfLiteTensorNumDims(input_tensor)
    input_shape = tuple(tflite.TfLiteTensorDim(input_tensor, i) for i in range(input_dims))
    tensor_type = tflite.TfLiteTensorType(input_tensor)

    # Map TFLite tensor type to NumPy dtype
    if tensor_type == 1: # kTfLiteFloat32
        input_dtype = np.float32
    elif tensor_type == 2: # kTfLiteInt32
        input_dtype = np.int32
    elif tensor_type == 3: # kTfLiteUInt8
        input_dtype = np.uint8
    else:
        raise ValueError(f"Unsupported tensor type: {tensor_type}")

    input_data = preprocess_image(image, input_shape, input_dtype)

    # Load input data to input tensor
    try:
        tflite.TfLiteTensorCopyFromBuffer(input_tensor, input_data.ctypes.data, input_data.nbytes)
    except Exception as e:
        print(f"Error copying input data to tensor: {e}")
        return [], 0
    # Run inference
    start_time = time.time()
    status = tflite.TfLiteInterpreterInvoke(interpreter)
    
    if status != 0:
        raise RuntimeError("TfLiteInterpreterInvoke failed!")
    else:
        print("Interpreter invoked successfully.")
    end_time = time.time()
    
    # Calculate and print duration
    inference_time = end_time - start_time
    print(f"Inference took {inference_time:.4f} seconds")
    
    # Get output tensor
    output_tensor = tflite.TfLiteInterpreterGetOutputTensor(interpreter, 0)
    output_tensor_type = tflite.TfLiteTensorType(output_tensor)
    print("Output tensor type:", output_tensor_type)

    # Prepare output tensor details
    output_dims = tflite.TfLiteTensorNumDims(output_tensor)
    print(f"this are the output_dims {output_dims}")
    output_shape = tuple(tflite.TfLiteTensorDim(output_tensor, i) for i in range(output_dims))
    print("Output shape:", output_shape)
    output_data = np.zeros(output_shape, dtype=np.uint8)
    
    # Load output data to output tensor
    tflite.TfLiteTensorCopyToBuffer(output_tensor, output_data.ctypes.data, output_data.nbytes)

    labels = load_labels(LABELS)
    predicted_index = np.argmax(output_data)
    predicted_label = labels[predicted_index]
    print("Predicted index:", predicted_index)
    print("Predicted label:", predicted_label)
    
    # Add Softmax function
    logits = output_data[0]
    probabilities = stable_softmax(logits)

    # Get top 4 predictions
    top_k = 4
    top_indices = np.argsort(probabilities)[::-1][:top_k]
    for i in top_indices:
        result = (labels[i], probabilities[i] * 100)
        results.append(result)

    clean(interpreter, model, options)

    return results, inference_time

def clean(interpreter, model, options):
    # Clean up
    tflite.TfLiteInterpreterDelete(interpreter)
    tflite.TfLiteModelDelete(model)
    tflite.TfLiteInterpreterOptionsDelete(options)

