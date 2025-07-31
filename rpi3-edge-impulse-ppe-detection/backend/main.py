# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------


# Python util packages
import asyncio
import json
import base64
from io import BytesIO, BufferedReader
import glob
from time import time
from collections import deque
from typing import Any, Awaitable

# Object detection packages
import requests
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFile
from qai_hub_models.models.gear_guard_net.app import BodyDetectionApp
from qai_hub_models.models.gear_guard_net.model import GearGuardNet

# Websocket packages
from websockets.asyncio.router import route
import websockets.exceptions
from werkzeug.routing import Map, Rule


CAM_MAX_X: int = 640
CAM_MAX_Y: int = 480
BOX_THICKNESS: int = 5
IMAGE_PATH: str = "snapshot.jpeg"
NUM_CLASSES = 2
MAX_INFERENCE_QUEUE_LEN: int = 1024


class ClassInfo:
    def __init__(self, id: int, name: str, color: str) -> None:
        self.id = id
        self.name = name
        self.color = color


CLASS_INFOS = [
    ClassInfo(0, "Hard Hat", "mediumblue"),
    ClassInfo(1, "Safety Vest", "magenta"),
]


class PPEItem:
    def __init__(self, class_info: ClassInfo, detected: bool, confidence: float, x1: int = None, y1: int = None, x2: int = None, y2: int = None) -> None:
        self.class_info = class_info
        self.detected = detected
        self.confidence = float(confidence)
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    @staticmethod
    def from_qai_hub_result(result: list[Any]):
        return PPEItem(
            class_info=CLASS_INFOS[int(result[0])],
            detected=True,
            confidence=result[5],
            x1=result[1],
            y1=result[2],
            x2=result[3],
            y2=result[4])

    @staticmethod
    def from_edge_impulse_result(result: dict[str, Any]):
        return PPEItem(
            class_info=CLASS_INFOS[0 if result["label"] == "hat" else 1],
            detected=True,
            confidence=result["value"],
            x1=result["x"],
            y1=result["y"],
            x2=result["x"] + result["width"],
            y2=result["y"] + result["height"])

    def to_json(self) -> dict:
        return {
            'name': self.class_info.name,
            'detected': self.detected,
            'confidence': self.confidence
        }


print("Starting server...")
app = BodyDetectionApp(GearGuardNet.from_pretrained())


# Adapted from: https://docs.edgeimpulse.com/docs/tools/edge-impulse-cli/cli-run-impulse#embedded-api-server
def get_snapshot(image_path: str, save_img: bool = False) -> BufferedReader:
    # Check if the webcam is opened successfully
    if not cam.isOpened():
        raise IOError("Cannot open webcam")

    # Capture a frame
    ret, frame = cam.read()

    # Check if frame is captured successfully
    if not ret:
        raise IOError("Cannot read frame")

    if save_img:
        # Save the frame as an image
        cv2.imwrite(image_path, frame)

    ret, img_encode = cv2.imencode('.jpeg', frame)
    if not ret:
        raise ValueError("Failed to encode image")

    # Convert to bytes and wrap in BufferedReader
    img_byteio = BytesIO(img_encode.tobytes())
    img_byteio.name = image_path
    reader = BufferedReader(img_byteio)

    return reader


def draw_box(image: ImageDraw, ppe_item: PPEItem):
    # Draw lines clockwise from bottom left
    image.line([(ppe_item.x1, ppe_item.y1), (ppe_item.x1, ppe_item.y2)],
               fill=ppe_item.class_info.color, width=BOX_THICKNESS)
    image.line([(ppe_item.x1, ppe_item.y2), (ppe_item.x2, ppe_item.y2)],
               fill=ppe_item.class_info.color, width=BOX_THICKNESS)
    image.line([(ppe_item.x2, ppe_item.y2), (ppe_item.x2, ppe_item.y1)],
               fill=ppe_item.class_info.color, width=BOX_THICKNESS)
    image.line([(ppe_item.x2, ppe_item.y1), (ppe_item.x1, ppe_item.y1)],
               fill=ppe_item.class_info.color, width=BOX_THICKNESS)


def img_to_base64(img: ImageFile):
    buffer = BytesIO()
    img.save(buffer, format="JPEG")

    return base64.b64encode(buffer.getvalue())


def get_queue_average(queue: deque) -> float:
    return sum(queue) / len(queue)


async def edge_impulse_handler(websocket: websockets.ServerConnection) -> Awaitable[None]:
    inference_time_queue = deque(maxlen=MAX_INFERENCE_QUEUE_LEN)
    inference_fps_queue = deque(maxlen=MAX_INFERENCE_QUEUE_LEN)

    while True:
        start = time()

        try:
            # Get an image from the camera
            image_buffered: BufferedReader = await asyncio.to_thread(get_snapshot, IMAGE_PATH)

            # Run inference on the image
            response: requests.Response = await asyncio.to_thread(requests.post, "http://localhost:8760/api/image", files={'file': image_buffered})
            if (response.status_code == 200):
                data = response.json()
                results = data["result"]["bounding_boxes"]

                # Select results with the highest confidence per class
                results = sorted(
                    results, key=lambda x: x["value"], reverse=True)

                # For drawing lines on the camera feed
                image_buffered.seek(0)
                img = Image.open(BytesIO(image_buffered.read()))
                draw = ImageDraw.Draw(img)

                CONFIDENCE_THRESHOLD: float = 0.7

                # Select only 1 inference result per class
                num_inferenced_classes = 0
                PPE_items: list[PPEItem] = [
                    PPEItem(CLASS_INFOS[i], False, -1) for i in range(NUM_CLASSES)]

                # Filter results
                for item in results:
                    if item["value"] >= CONFIDENCE_THRESHOLD:
                        current_class = 0 if item["label"] == "hat" else 1

                        # If we have not detected this class yet
                        if PPE_items[current_class].confidence == -1:
                            # Nicely format the inference results so we can send to the frontend
                            PPE_item = PPEItem.from_edge_impulse_result(
                                item)

                            PPE_items[current_class] = PPE_item

                            draw_box(draw, PPE_item)

                            num_inferenced_classes += 1

                            if num_inferenced_classes >= NUM_CLASSES:
                                break

                # Calculate inference times
                inference_time = time() - start
                inference_fps = 1 / inference_time

                inference_time_queue.append(inference_time)
                inference_fps_queue.append(inference_fps)

                # Send inference results
                data_to_send = {
                    'data': list(map(lambda item: item.to_json(), PPE_items)),
                    'image': img_to_base64(img).decode("utf-8"),
                    'inference_time': {
                        'value': inference_time,
                        'average': get_queue_average(inference_time_queue)
                    },
                    'inference_fps': {
                        'value': inference_fps,
                        'average': get_queue_average(inference_fps_queue),
                    },
                }

                await websocket.send(json.dumps(data_to_send))

        except websockets.exceptions.ConnectionClosed:
            break


async def qai_hub_handler(websocket: websockets.ServerConnection) -> Awaitable[None]:
    inference_time_queue = deque(maxlen=MAX_INFERENCE_QUEUE_LEN)
    inference_fps_queue = deque(maxlen=MAX_INFERENCE_QUEUE_LEN)

    while True:
        start = time()

        try:
            # Get an image from the camera
            image_buffered: BufferedReader = await asyncio.to_thread(get_snapshot, IMAGE_PATH, True)

            # Run inference on the image
            result: np.ndarray = app.detect(
                IMAGE_PATH, 320, 192, 0.80)

            # Filter out incorrect inference results
            filter_mask = (
                (result[:, 1] >= 0) & (result[:, 2] >= 0) &
                (result[:, 3] >= 0) & (result[:, 4] >= 0) &
                (result[:, 1] <= CAM_MAX_X) & (result[:, 3] <= CAM_MAX_X) &
                (result[:, 2] <= CAM_MAX_Y) & (result[:, 4] <= CAM_MAX_Y)
            )
            result = result[filter_mask]

            # Select results with the highest confidence per class
            result = result[np.argsort(result[:, 4])[::-1]]

            # For drawing lines on the camera feed
            image_buffered.seek(0)
            img = Image.open(BytesIO(image_buffered.read()))
            draw = ImageDraw.Draw(img)

            # Select only 1 inference result per class
            num_inferenced_classes = 0
            PPE_items: list[PPEItem] = [
                PPEItem(CLASS_INFOS[i], False, -1) for i in range(NUM_CLASSES)]

            for item in result:
                current_class = int(item[0])  # Either 0 or 1

                # If we have not detected this class yet
                if PPE_items[current_class].confidence == -1:
                    # Nicely format the inference results so we can send to the frontend
                    PPE_item = PPEItem.from_qai_hub_result(item)

                    PPE_items[current_class] = PPE_item

                    draw_box(draw, PPE_item)

                    num_inferenced_classes += 1

                    if num_inferenced_classes >= NUM_CLASSES:
                        break

            # Calculate inference times
            inference_time = time() - start
            inference_fps = 1 / inference_time

            inference_time_queue.append(inference_time)
            inference_fps_queue.append(inference_fps)

            # Send inference results
            data_to_send = {
                'data': list(map(lambda item: item.to_json(), PPE_items)),
                'image': img_to_base64(img).decode("utf-8"),
                'inference_time': {
                    'value': inference_time,
                    'average': get_queue_average(inference_time_queue)
                },
                'inference_fps': {
                    'value': inference_fps,
                    'average': get_queue_average(inference_fps_queue),
                },
            }

            await websocket.send(json.dumps(data_to_send))

        except websockets.exceptions.ConnectionClosed:
            break


url_map = Map([
    Rule("/edge-impulse", endpoint=edge_impulse_handler),
    Rule("/qai-hub", endpoint=qai_hub_handler),
])


async def main():
    # Initialize the webcam
    global cam

    # Find the correct camera path
    video_devices = glob.glob('/dev/video*')
    for cam_path in video_devices:
        cam = cv2.VideoCapture(cam_path)

        if cam.isOpened():
            break

    if not cam.isOpened():
        raise IOError(
            "Cannot open webcam. Make sure a supported camera is properly connected")

    stop = asyncio.get_running_loop().create_future()

    async with route(url_map, "0.0.0.0", 8765) as server:
        print("Started server!")

        await stop

    # Release the camera
    cam.release()

if __name__ == "__main__":
    asyncio.run(main())
