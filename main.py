# Example program for the T-WATCH S3

import array
import random
import time
from machine import Pin, SPI, SoftI2C
import framebuf

import st7789_base, st7789_ext
from axp2101 import AXP2101
import bma423

import emltrees
from microfont import MicroFont

font = MicroFont("victor:B:32.mfnt", cache_index=False)

fb_width = 240
fb_height = 240

def test_trees():

    model = emltrees.new(5, 30, 5)
    #print(model)
    with open('xor_model.csv') as f:
        emltrees.load_model(model, f)

    a = array.array('f', [0.0, 1.0])
    out = model.predict(a)
    print(out)


def setup():

    # Setup the PMU chip.
    twatch_pmu = AXP2101()
    twatch_pmu.twatch_s3_poweron()

    # Power on the display backlight.
    bl = Pin(45,Pin.OUT)
    bl.on()

    print("Backlight on")

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

    return twatch_pmu, display

def update_display(display, text):

    start = time.ticks_ms()
    bg_color = display.fb_color(0, 0, 0)
    display.fb.fill(bg_color)

    text_color = display.fb_color(255, 255, 255)

    fb = display.fb
    color = 0xffffff # Color must be in the framebuffer color mode format.
    angle = 0
    font.write(text, fb, framebuf.RGB565, fb_width, fb_height, 150, 150, color,
               rot=angle, x_spacing=0, y_spacing=0)

    display.show()
    elapsed = time.ticks_ms() - start
    print("Ticks per screen fill:", elapsed)


def main():

    test_trees() # XXX: fails if done after setup!?

    pmu, display = setup()
    
    print("[AXP2101] Battery voltage is", pmu.get_battery_voltage())
    print("Display ON")

    #rtc = machine.RTC()
    
    # accelerometer
    i2c = SoftI2C(sda=10,scl=11)
    sensor = bma423.BMA423(i2c, addr=0x19)


    while True:

        text = str(int(time.ticks_ms()/1000))
        acc = sensor.get_xyz()
        t = 0
        t = sensor.get_temperature()
        print('acc XYZ', acc, t)
        
        print('set text', text)
        update_display(display, text)
    
        # Pause 2 seconds.
        time.sleep(2)

main()
