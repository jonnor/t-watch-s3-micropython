# Example program for the T-WATCH S3

import emltrees
import array

import random

from machine import Pin, SPI
import st7789_base, st7789_ext
import time
from axp2101 import AXP2101
import framebuf

from microfont import MicroFont

font = MicroFont("victor:B:32.mfnt", cache_index=False)

def test_trees():

    model = emltrees.new(5, 30, 5)
    with open('xor_model.csv') as f:
        emltrees.load_model(model, f)

    a = array.array('f', [0.0, 1.0])
    out = model.predict(a)
    print(out)


def main():
    # Setup the PMU chip.
    twatch_pmu = AXP2101()
    twatch_pmu.twatch_s3_poweron()
    print("[AXP2101] Battery voltage is", twatch_pmu.get_battery_voltage())

    # Power on the display backlight.
    bl = Pin(45,Pin.OUT)
    bl.on()

    print("Backlight on")

    test_trees()

    fb_width = 240
    fb_height = 240
    # Our display does not have a MISO pin, but the MicroPython
    # SPI implementation does not allow to avoid specifying one, so
    # we use just a not used pin in the device.
    spi = SPI(1, baudrate=40000000, phase=0, polarity=1, sck=18, mosi=13, miso=12)
    display = st7789_ext.ST7789(
        spi,
        fb_width, fb_height,
        reset=False,
        dc=Pin(38, Pin.OUT),
        cs=Pin(12, Pin.OUT),
    )
    display.init(landscape=False,mirror_y=True,mirror_x=True,inversion=True)
    display.enable_framebuffer()

    print("Display ON")

    print("displaying random colors")
    while True:
        start = time.ticks_ms()
        bg_color = display.fb_color(0, 0, 0)
        display.fb.fill(bg_color)

        text_color = display.fb_color(255, 255, 255)
        #display.upscaled_text(00, 00, '123', fgcolor=b'0xffffff', upscaling=4)

        fb = display.fb
        color = 0xffffff # Color must be in the framebuffer color mode format.
        angle = 0
        text = str(int(time.ticks_ms()/1000))
        font.write(text, fb, framebuf.RGB565, fb_width, fb_height, 150, 150, color,
                   rot=angle, x_spacing=0, y_spacing=0)

        display.show()
        elapsed = time.ticks_ms() - start
        print("Ticks per screen fill:", elapsed)

        # Pause 2 seconds.
        time.sleep(2)

main()
