# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

import os, sys, math, random, requests
from pathlib import Path
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import InputLayer, Dropout, Dense
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping

sys.path.append('./resources/libraries')
import ei_tensorflow.training

BEST_MODEL_PATH = os.getenv("BEST_MODEL_PATH", "best_model.keras")

INPUT_SHAPE = (96, 96, 3)
BATCH_SIZE   = args.batch_size or 128
EPOCHS       = args.epochs or 10
LEARNING_RATE= args.learning_rate or 0.0005
ENSURE_DETERMINISM = args.ensure_determinism  # <-- your pattern

if ENSURE_DETERMINISM:
    SEED = 1337
    random.seed(SEED)
    tf.random.set_seed(SEED)
    os.environ["TF_DETERMINISTIC_OPS"] = "1"

WEIGHTS_PATH = './transfer-learning-weights/keras/mobilenet_v2_weights_tf_dim_ordering_tf_kernels_1.0_96_no_top.h5'
use_keras_imagenet = False

root_url = 'https://cdn.edgeimpulse.com/'
p = Path(WEIGHTS_PATH)
if not p.exists():
    try:
        print(f"Pretrained weights {WEIGHTS_PATH} unavailable; downloading...")
        p.parent.mkdir(parents=True, exist_ok=True)
        r = requests.get(root_url + WEIGHTS_PATH[2:], timeout=60)
        r.raise_for_status()
        p.write_bytes(r.content)
        print("Downloaded OK\n")
    except Exception as e:
        print(f"[WARN] EI weights download failed ({e}). Falling back to keras 'imagenet'.\n")
        use_keras_imagenet = True

if use_keras_imagenet:
    base_model = tf.keras.applications.MobileNetV2(
        input_shape=INPUT_SHAPE, alpha=1.0, include_top=False, weights="imagenet", pooling='avg'
    )
else:
    base_model = tf.keras.applications.MobileNetV2(
        input_shape=INPUT_SHAPE, alpha=1.0, include_top=False, weights=WEIGHTS_PATH, pooling='avg'
    )
base_model.trainable = False

model = Sequential([
    InputLayer(input_shape=INPUT_SHAPE, name='x_input'),
    base_model,
    Dropout(0.1),
    Dense(classes, activation='softmax')
])

def augment_image(image, label):
    image = tf.image.random_flip_left_right(image)
    resize_factor = random.uniform(1, 1.2)
    new_height = math.floor(resize_factor * INPUT_SHAPE[0])
    new_width  = math.floor(resize_factor * INPUT_SHAPE[1])
    image = tf.image.resize_with_crop_or_pad(image, new_height, new_width)
    image = tf.image.random_crop(image, size=INPUT_SHAPE)
    image = tf.image.random_brightness(image, max_delta=0.2)
    return image, label

train_dataset = train_dataset.map(
    augment_image,
    num_parallel_calls=tf.data.AUTOTUNE if not ENSURE_DETERMINISM else 1
)

if not ENSURE_DETERMINISM:
    train_dataset = train_dataset.shuffle(buffer_size=BATCH_SIZE*4)

prefetch_policy = 1 if ENSURE_DETERMINISM else tf.data.AUTOTUNE
train_dataset      = train_dataset.batch(BATCH_SIZE, drop_remainder=False).prefetch(prefetch_policy)
validation_dataset = validation_dataset.batch(BATCH_SIZE, drop_remainder=False).prefetch(prefetch_policy)

callbacks.append(ModelCheckpoint(
    BEST_MODEL_PATH, monitor="val_accuracy", mode="max", save_best_only=True, verbose=1
))
callbacks.append(EarlyStopping(
    monitor="val_loss", patience=4, restore_best_weights=True, verbose=1
))
callbacks.append(BatchLoggerCallback(
    BATCH_SIZE, train_sample_count, epochs=EPOCHS, ensure_determinism=ENSURE_DETERMINISM
))

model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=LEARNING_RATE),
    loss=tf.keras.losses.CategoricalCrossentropy(label_smoothing=0.05),
    metrics=['accuracy']
)

model.fit(
    train_dataset,
    validation_data=validation_dataset,
    epochs=EPOCHS,
    verbose=2,
    callbacks=callbacks,
    class_weight=ei_tensorflow.training.get_class_weights(Y_train)
)

print('\nInitial training done.\n', flush=True)

print('Fine-tuning best model for 10 epochs...', flush=True)
model = ei_tensorflow.training.load_best_model(BEST_MODEL_PATH)

FINE_TUNE_EPOCHS = 10
FINE_TUNE_PERCENTAGE = 65
model_layer_count = len(model.layers)
fine_tune_from = math.ceil(model_layer_count * ((100 - FINE_TUNE_PERCENTAGE) / 100))

model.trainable = True
for layer in model.layers[:fine_tune_from]:
    layer.trainable = False

model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=4.5e-5),
    loss=tf.keras.losses.CategoricalCrossentropy(label_smoothing=0.05),
    metrics=['accuracy']
)

model.fit(
    train_dataset,
    epochs=FINE_TUNE_EPOCHS,
    verbose=2,
    validation_data=validation_dataset,
    callbacks=callbacks,
    class_weight=ei_tensorflow.training.get_class_weights(Y_train)
)
