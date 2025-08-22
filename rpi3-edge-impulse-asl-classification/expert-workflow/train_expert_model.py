# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

import argparse
import pathlib
import tensorflow as tf
from tensorflow.keras import layers, models

# Argument parsing
parser = argparse.ArgumentParser(description="Train ASL classifier with MobileNetV2")
parser.add_argument("--data_dir", type=str, required=True, help="Path to training data directory")
parser.add_argument("--weights_path", type=str, required=True, help="Path to MobileNetV2 weights file")
parser.add_argument("--output_dir", type=str, default="expert-workflow", help="Directory to save outputs")
parser.add_argument("--epochs", type=int, default=10, help="Number of training epochs")
parser.add_argument("--batch_size", type=int, default=128, help="Batch size")
args = parser.parse_args()

# Constants
IMG_SIZE = (160, 160)
ALPHA = 1
CLASS_ORDER = ['A', 'B', 'C', 'D', "E", 'F', 'G', "H", "I", 'J', "K", "L", 'M', 'N', "Nothing", "O", "P", "Q", "R", "S", "Space", "T", "U", "V", 'W', 'X', 'Y', "Z"]
num_classes = len(CLASS_ORDER)

# Paths
DATA_DIR = pathlib.Path(args.data_dir)
WEIGHTS_PATH = pathlib.Path(args.weights_path)
OUTDIR = pathlib.Path(args.output_dir)
OUTDIR.mkdir(parents=True, exist_ok=True)

# Load datasets
train_ds = tf.keras.preprocessing.image_dataset_from_directory(
    DATA_DIR,
    labels='inferred',
    label_mode='int',
    class_names=CLASS_ORDER,
    validation_split=0.2,
    subset='training',
    seed=1337,
    image_size=IMG_SIZE,
    batch_size=args.batch_size,
    shuffle=True
)

val_ds = tf.keras.preprocessing.image_dataset_from_directory(
    DATA_DIR,
    labels='inferred',
    label_mode='int',
    class_names=CLASS_ORDER,
    validation_split=0.2,
    subset='validation',
    seed=1337,
    image_size=IMG_SIZE,
    batch_size=args.batch_size,
    shuffle=False
)

AUTOTUNE = tf.data.AUTOTUNE
train_ds = train_ds.cache().prefetch(AUTOTUNE)
val_ds = val_ds.cache().prefetch(AUTOTUNE)

# Model
inputs = layers.Input(shape=(160, 160, 3), name="image")
x = layers.RandomFlip("horizontal")(inputs)
x = layers.RandomRotation(0.05)(x)
x = layers.RandomZoom(0.10)(x)
x = layers.RandomContrast(0.10)(x)
x = layers.Rescaling(1.0/127.5, offset=-1)(x)

base = tf.keras.applications.MobileNetV2(
    input_shape=(160, 160, 3),
    include_top=False,
    weights=str(WEIGHTS_PATH),
    alpha=ALPHA
)
base.trainable = False

x = base(x, training=False)
x = layers.GlobalAveragePooling2D()(x)
x = layers.Dropout(0.2)(x)
outputs = layers.Dense(num_classes, activation="softmax")(x)
model = models.Model(inputs, outputs)

model.compile(
    optimizer=tf.keras.optimizers.Adam(0.0005),
    loss=tf.keras.losses.SparseCategoricalCrossentropy(),
    metrics=["accuracy"]
)

callbacks = [
    tf.keras.callbacks.EarlyStopping(monitor="val_accuracy", patience=5, restore_best_weights=True),
    tf.keras.callbacks.ModelCheckpoint(OUTDIR / "best_stage1.keras", save_best_only=True, monitor='val_accuracy'),
    tf.keras.callbacks.ReduceLROnPlateau(monitor='val_loss', patience=2, factor=0.2)
]

model.fit(train_ds, validation_data=val_ds, epochs=args.epochs, callbacks=callbacks, verbose=2)

# Fine-tuning
base.trainable = True
cut = int(len(base.layers) * 0.8)
for layer in base.layers[:cut]:
    layer.trainable = False

model.compile(
    optimizer=tf.keras.optimizers.Adam(0.0005),
    loss=tf.keras.losses.SparseCategoricalCrossentropy(),
    metrics=["accuracy"]
)

callbacks_ft = [
    tf.keras.callbacks.EarlyStopping(monitor="val_accuracy", patience=6, restore_best_weights=True),
    tf.keras.callbacks.ModelCheckpoint(OUTDIR / "best_stage2.keras", save_best_only=True, monitor='val_accuracy'),
    tf.keras.callbacks.ReduceLROnPlateau(monitor='val_loss', patience=2, factor=0.2)
]

model.fit(train_ds, validation_data=val_ds, epochs=args.epochs, callbacks=callbacks_ft, verbose=2)

# Save model
model.save(OUTDIR / "model.keras")
model.save(OUTDIR / "saved_model_run", save_format="tf")
