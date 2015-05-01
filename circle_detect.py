#!/usr/bin/python

# Distance estimation

import PIL
import SimpleCV
import io
import math
import picamera
import picamera.array
import signal
import sys
import time


class PictureWrapper(object):
    def __init__(self, resolution=None):
        self._camera = picamera.PiCamera()
        scale = 5

        if resolution is not None:
            self._camera.resolution = resolution
        else:
            self._camera.resolution = (320, 240)

        self._camera.start_preview()

    def get_image(self):
        try:
            with picamera.array.PiRGBArray(self._camera) as stream:
                self._camera.capture(stream, format='rgb', use_video_port=True)
                pil = PIL.Image.fromarray(stream.array, mode='RGB')
                image = SimpleCV.Image(pil)

            return image
        # This usually happens when we try to shut down the program using
        # SIGINT; just ignore it
        except (picamera.PiCameraValueError, picamera.PiCameraAlreadyRecording):
            return None

class Watch(object):
    # Does image processing to determine the distance and direction to a target
    HORIZONTAL_VIEWPORT_D = 55 * 2.0
    HORIZONTAL_VIEWPORT_R = math.radians(HORIZONTAL_VIEWPORT_D)
    ORANGE_DUCT_HUE = 7.0
    CD_DIAMETER_M = .110

    def __init__(self, picture_wrapper):
        self._wrapper = picture_wrapper
        image = picture_wrapper.get_image()
        self._r_per_pixel = self.HORIZONTAL_VIEWPORT_R / image.width
        self._d_per_pixel = self.HORIZONTAL_VIEWPORT_D / image.width
        # From manual testing, 100 seems to be a good minsize for 320x240
        self._min_size = int(image.width * image.height / 320.0 / 240.0 * 100.0)

    def _get_distance_m_direction_d(self, image):
        #Returns the distance in meters and the direction in degrees to the target from an image
        if image is None:
            return (None, None)
        crop_y = int(image.height * .3)
        crop_height = int(image.height * .5)
        def crop(img):
            return img.crop(0, crop_y, image.width, crop_height)

        # The hue distance to orange from black is pretty low, so we need
        # to create a black mask and subtract those parts first
        # TODO: I don't know why, but cropping the image first and then doing
        # distance produces an image with weird streaks instead of anything
        # correct, so we can't just crop it immediately
        #image = crop(image)
        #black_mask = image.colorDistance(SimpleCV.Color.BLACK).binarize()
        #distance = image.hueDistance(self.ORANGE_DUCT_HUE).invert().binarize(220).invert()
        black_mask = crop(image.colorDistance(SimpleCV.Color.BLACK)).binarize()
        distance = crop(image).hueDistance(self.ORANGE_DUCT_HUE).invert().binarize(220).invert()
        blobs = (distance - black_mask).findBlobs(
            minsize=self._min_size,
            maxsize=(distance.width * distance.height // 4)
        )
        if blobs is not None:
            circles = [b for b in blobs if b.isCircle(0.5)]
            if len(circles) == 0:
                return (None, None)
            # The circles should already be sorted by area ascending
            angle_r = circles[-1].radius() * self._r_per_pixel
            distance_m = self.CD_DIAMETER_M / math.tan(angle_r)

            mid_circle_x = circles[-1].x + circles[-1].radius() * 0.5
            angle_d = (mid_circle_x - distance.width * 0.5) * self._d_per_pixel
            if angle_d < 0.0:
                angle_d += 360.0
            return (distance_m, angle_d)

        return (None, None)

    def get_distance_m_direction_d(self):
        # Returns the distance in meters and the direction in degrees to the target
        image = self._wrapper.get_image()
        return self._get_distance_m_direction_d(image)


START       = None
ITERATIONS  = None
END         = None


def terminate(signal_number, stack_frame):
    print(
        '{iterations} iterations took {seconds} seconds, fps = {fps}'.format(
            iterations=ITERATIONS,
            seconds=(END - START),
            fps=(ITERATIONS / (END - START))
        )
    )
    sys.exit()

def main():
    signal.signal(signal.SIGINT, terminate)
    picture_wrapper = PictureWrapper()
    watch = Watch(picture_wrapper)

    global START
    global ITERATIONS
    global END

    ITERATIONS = 0
    START = time.time()
    while True:
        distance_m, angle_d = watch.get_distance_m_direction_d()
        print(
            'Distance: {distance_m} m {distance_in} in, angle: {angle}'.format(
                distance_m=distance_m,
                distance_in=(None if distance_m is None else distance_m * 39.3700787),
                angle=angle_d
            )
        )
        ITERATIONS += 1
        END = time.time()


if __name__ == '__main__':
    main()