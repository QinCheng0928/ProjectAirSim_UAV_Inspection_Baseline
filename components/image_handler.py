import os
import cv2
import numpy as np
from datetime import datetime
from projectairsim.utils import unpack_image, projectairsim_log
from projectairsim.image_utils import ImageDisplay
from config.settings import SAVE_PATH, VIDEO_FPS, SUBWIN_WIDTH, SUBWIN_HEIGHT

class ImageHandler:
    def __init__(self, client, num_subwim=3, display_enabled=True, save_enabled=False):
        self.client = client
        self.display_enabled = display_enabled
        self.save_enabled = save_enabled

        self.image_display = ImageDisplay(
            num_subwin=num_subwim,
            screen_res_x=2560,
            screen_res_y=1440,
            subwin_width=SUBWIN_WIDTH,
            subwin_height=SUBWIN_HEIGHT
        )
        self.video_writers = {}
        self.topics = {}

    def start(self):
        if self.display_enabled:
            self.image_display.start()

    def stop(self):
        if self.display_enabled:
            self.image_display.stop()
        self.close_writers()

    def register_window(self, name, subwin_idx):
        self.image_display.add_image(name, subwin_idx=subwin_idx, 
                                   resize_x=SUBWIN_WIDTH, resize_y=SUBWIN_HEIGHT)

    def subscribe_camera(self, sensor_topic, window_name):
        self.client.subscribe(sensor_topic, 
                            lambda topic, msg: self._on_image_received(msg, window_name))

    def _on_image_received(self, image_msg, topic_name):
        if self.display_enabled:
            self.image_display.receive(image_msg, topic_name)

        if self.save_enabled:
            frame = unpack_image(image_msg)
            if frame is None:
                return
            writer = self._get_or_create_writer(topic_name, frame)
            bgr_frame = self._frame_to_bgr(frame)
            if bgr_frame is not None:
                writer.write(bgr_frame)

    def _frame_to_bgr(self, frame):
        if frame is None:
            return None
        if frame.ndim == 2:
            depth_map = np.nan_to_num(frame)
            if depth_map.dtype != np.uint8:
                depth_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)
                depth_uint8 = depth_norm.astype(np.uint8)
            else:
                depth_uint8 = depth_map
            return cv2.cvtColor(depth_uint8, cv2.COLOR_GRAY2BGR)
        elif frame.ndim == 3 and frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def _get_or_create_writer(self, topic_name, sample_frame):
        if topic_name in self.video_writers:
            return self.video_writers[topic_name]

        os.makedirs(SAVE_PATH, exist_ok=True)
        height, width = sample_frame.shape[:2]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = "".join(c for c in str(topic_name) if c.isalnum() or c in ('_', '-')).rstrip()
        video_path = os.path.join(SAVE_PATH, f"{safe_name}_{timestamp}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(video_path, fourcc, VIDEO_FPS, (width, height))
        self.video_writers[topic_name] = writer
        projectairsim_log().info(f"Created video writer for {topic_name}: {video_path}")
        return writer

    def close_writers(self):
        for name, writer in list(self.video_writers.items()):
            if writer is not None:
                writer.release()
        self.video_writers.clear()