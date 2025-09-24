# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

import cv2
import gi
import numpy as np
import time
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib, GdkPixbuf
import ai_edge_litert.interpreter as tflite
import qliFunctions

# ========= Constants =========
TF_MODEL = "googlenet_quantized.tflite"
LABELS = "imagenet_labels.txt"
DELEGATE_PATH = "libQnnTFLiteDelegate.so"

# ========= Helper Functions =========
def stable_softmax(logits):
    # Convert logits to float64 for higher precision
    logits = logits.astype(np.float32)
    
    # Subtract the maximum logit to prevent overflow
    shifted_logits = logits - np.max(logits)
    
    # Clip the shifted logits to a safe range to prevent overflow in exp
    shifted_logits = np.clip(shifted_logits, -500, 500)
    
    # Calculate the exponentials and normalize
    exp_scores = np.exp(shifted_logits)
    probabilities = exp_scores / np.sum(exp_scores)
    
    return probabilities

# Load labels from file
def load_labels(label_path):
    with open(label_path, 'r') as f:
        return [line.strip() for line in f.readlines()]

def resizeImage(pixbuf):
    original_width = pixbuf.get_width()
    original_height = pixbuf.get_height()

    # Target display size
    max_width = 800
    max_height = 600

    # Calculate new size preserving aspect ratio
    scale = min(max_width / original_width, max_height / original_height)
    new_width = int(original_width * scale)
    new_height = int(original_height * scale)

    return new_width, new_height

# Load and preprocess input image
def preprocess_image(image_path, input_shape, input_dtype):
    # Read the image using OpenCV
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Failed to load image at {image_path}")
    # Convert BGR to RGB
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # Resize the image to the desired input shape
    img = cv2.resize(img, (input_shape[2], input_shape[1]))
    # Convert to the desired data type
    img = img.astype(input_dtype)
    # Add batch dimension
    img = np.expand_dims(img, axis=0)
    
    return img

# ====== Inference Function ======
def runInference(image, use_delegate):
    results = []
    workingOS = qliFunctions.checkOS()
    #Check OS version
    if (workingOS == "Ubuntu"):
        print(f"Running on {workingOS} using Delegate:{use_delegate}")
        if use_delegate:
            try:
                # Load the QNN delegate library
                delegate_options = { 'backend_type': 'htp' }
                delegate = tflite.load_delegate(DELEGATE_PATH, delegate_options)
                
                # Load the TFLite model
                model = tflite.Interpreter(model_path=TF_MODEL, experimental_delegates=[delegate])
                print("INFO: Loaded QNN delegate with HTP backend")
            except Exception as e:
                print(f"WARNING: Failed to load QNN delegate: {e}")
                print("INFO: Continuing without QNN delegate")
                model = tflite.Interpreter(model_path=TF_MODEL)   
        else:
            model = tflite.Interpreter(model_path=TF_MODEL)  
       
        model.allocate_tensors()

        # Get and Prepare input 
        input_details = model.get_input_details()
        input_shape = input_details[0]['shape']
        input_dtype = input_details[0]['dtype']
        input_data = preprocess_image(image, input_shape, input_dtype)
        
        # Load input data to input tensor
        model.set_tensor(input_details[0]['index'], input_data)
        model.get_signature_list()
        
        # Run inference
        try:
            start_time = time.time()
            model.invoke()
            end_time = time.time()
            print("Interpreter invoked successfully.")
        except Exception as e:
            print(f"Error during model invocation: {e}")
            return []

        # Calculate and print duration
        inference_time = end_time - start_time

        # Prepare output tensor details
        output_details = model.get_output_details()

        # Load output data to output tensor
        output_data = model.get_tensor(output_details[0]['index'])

        # Load labels and get prediction
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
    
    elif (workingOS == "QualcommLinux"):
        results, inference_time = qliFunctions.inferenceQLI(image, use_delegate, TF_MODEL, DELEGATE_PATH, LABELS)   

    return results, inference_time

# ====== GTK GUI Classes ======
class MainWindow(Gtk.Window):
    def __init__(self):
        super().__init__(title="RB3 Gen 2 Image Classification")
        self.set_default_size(800, 600)
        self.imageFilepath = ""
        # Main layout
        self.mainBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        self.mainBox.set_margin_top(10)
        self.mainBox.set_margin_bottom(10)
        self.mainBox.set_margin_start(10)
        self.mainBox.set_margin_end(10)
        self.add(self.mainBox)
        
        # Main Window Image setup with fallback
        self.image = Gtk.Image()
        try:
            MAIN_IMAGE = "MainWindowPic.jpg"
            self.image.set_from_file(MAIN_IMAGE)         
        except Exception as e:
            print("Error loading main image:", e)
            self.image.set_from_icon_name("image-missing", Gtk.IconSize.DIALOG)

        self.mainBox.pack_start(self.image, True, True, 0)

        # Set up a new box to add results and and file button
        self.infoBox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        
        # Radio button to select Delegate
        delegate_label = Gtk.Label(label="Select Inference Mode:")
        self.infoBox.pack_start(delegate_label, False, False, 10)

        self.cpu_radio = Gtk.RadioButton.new_with_label_from_widget(None, "CPU")
        self.delegate_radio = Gtk.RadioButton.new_with_label_from_widget(self.cpu_radio, "Delegate")

        self.infoBox.pack_start(self.cpu_radio, False, False, 0)
        self.infoBox.pack_start(self.delegate_radio, False, False, 0)
        
        # Radio button signal
        self.cpu_radio.connect("toggled", self.on_radio_toggled)
        self.delegate_radio.connect("toggled", self.on_radio_toggled)

        # Open file button
        open_button = Gtk.Button(label="Select Image")
        open_button.connect("clicked", self.on_open_file_clicked)
        self.infoBox.pack_start(open_button, False, True, 10)

        # Reprocess Image
        reprocess_button = Gtk.Button(label="Reprocess Image")
        reprocess_button.connect("clicked", self.on_reprocess_image_clicked)
        self.infoBox.pack_start(reprocess_button, False, True, 10)

        # Classification results
        self.results = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.infoBox.pack_start(self.results, True, True, 0)
        self.mainBox.pack_start(self.infoBox, True, True, 0)

    def use_delegate(self):
        return self.delegate_radio.get_active()

    def on_radio_toggled(self, button):
        if button.get_active():
            print(f"Selected option: {button.get_label()}")

    def process_file(self, filepath): 
        try:
            # Resize Image
            pixbuf = GdkPixbuf.Pixbuf.new_from_file(filepath)
            new_width, new_height = resizeImage(pixbuf)
            scaled_pixbuf = pixbuf.scale_simple(new_width, new_height, GdkPixbuf.InterpType.BILINEAR)
            
            # Replace the image with new image
            self.image.set_from_pixbuf(scaled_pixbuf)
           
            # Run Inference
            use_delegate = self.use_delegate()
            print("delegate: " , use_delegate)
            options, inference_time = runInference(filepath, use_delegate)

            # Clear result box
            for child in self.results.get_children():
                self.results.remove(child)
            
            # Set up predictions
            for label, percent in options:
                textBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
                barBox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
                text = Gtk.Label(label=label, xalign=0)
                text.set_size_request(100, -1) 
                
                bar = Gtk.ProgressBar()
                bar.set_fraction(percent / 100.0)
                bar.set_text(f"{percent:.2f}%")
                bar.set_show_text(True)
                
                textBox.pack_start(text, False, False, 0)
                barBox.pack_start(bar, True, True, 0)
            
                self.results.pack_start(textBox, False, False, 0)
                self.results.pack_start(barBox, False, False, 0)
                self.results.show_all()
            
            # Add inference time label
            time_label = Gtk.Label(label=f"Inference Time : {inference_time:.4f} s")
            self.results.pack_start(time_label, False, False, 50)
            self.results.show_all()
        except Exception as e:
            print("Error reading file:", e)

    def on_open_file_clicked(self, widget):
        dialog = FileBrowser()
        selected_file = dialog.run_and_get_file()
        self.imageFilepath = selected_file
        if selected_file:
            self.process_file(selected_file)
 
    def on_reprocess_image_clicked(self, widget):
        self.process_file(self.imageFilepath)

    def on_destroy(self, widget):
        Gtk.main_quit()

class FileBrowser(Gtk.FileChooserDialog):
    def __init__(self):
        super().__init__(title="Choose an image", action=Gtk.FileChooserAction.OPEN)
        self.add_buttons(Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL, Gtk.STOCK_OPEN, Gtk.ResponseType.OK)

    def run_and_get_file(self):
        response = super().run()
        if response == Gtk.ResponseType.OK:
            print("Selected file:", self.get_filename())
            self.selected_file = self.get_filename()            
        self.destroy()
        return self.selected_file

# === Main Entry Point ===
def main():
    app = MainWindow()
    app.connect("destroy", Gtk.main_quit)
    app.show_all()
    Gtk.main()

if __name__ == "__main__":
    success, _ = Gtk.init_check()
    if not success:
        print("GTK could not be initialized. Check environmental variables")
        exit(1)

    main() 
