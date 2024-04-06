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


def decode_acceleration(sensor, inp, out):
    # inp is the raw bytes for a set of acceleration samples, typically read from FIFO
    # NOTE: must be without FIFO headers!
    
    bytes_per_sample = 2
    axes = 3
    n_samples = len(inp) // (bytes_per_sample*axes)
    for sample_no in range(0, n_samples):
        sample_start = sample_no*bytes_per_sample*axes
        rawdata = memoryview(inp)[sample_start:]
        acc_x = (rawdata[0] >> 4) | (rawdata[1] << 4)
        acc_y = (rawdata[2] >> 4) | (rawdata[3] << 4)
        acc_z = (rawdata[4] >> 4) | (rawdata[5] << 4)
        acc_x = sensor.convert_to_int12(acc_x)
        acc_y = sensor.convert_to_int12(acc_y)
        acc_z = sensor.convert_to_int12(acc_z)
        acc_x = sensor.normalize_reading(acc_x)
        acc_y = sensor.normalize_reading(acc_y)
        acc_z = sensor.normalize_reading(acc_z)
        out[(sample_no*axes)+0] = acc_x
        out[(sample_no*axes)+1] = acc_y
        out[(sample_no*axes)+2] = acc_z
    

def main():

    test_trees() # XXX: fails if done after setup!?

    pmu, display = setup()
    
    print("[AXP2101] Battery voltage is", pmu.get_battery_voltage())
    print("Display ON")

    #rtc = machine.RTC()
    
    # accelerometer
    i2c = SoftI2C(sda=10,scl=11)
    sensor = bma423.BMA423(i2c, addr=0x19)

    sensor.set_accelerometer_freq(50)
    sensor.fifo_enable()
    sensor.fifo_clear()

    accel_samples = 25
    accel_buffer = bytearray(accel_samples*3*2) # 3 axes, 2 bytes each
    accel_array = array.array('f', list(range(0, accel_samples*3)))

    while True:

        text = str(int(time.ticks_ms()/1000))
        #acc = sensor.get_xyz()
        t = 0
        t = sensor.get_temperature()
        #print('acc XYZ', acc, t)
        #print('temp', t)
        
        fifo_level = sensor.fifo_level()
        #print('FIFO level', time.ticks_ms(), fifo_level)
        if fifo_level >= len(accel_buffer):
            read_start = time.ticks_ms()
            sensor.fifo_read(accel_buffer)
            read_dur = time.ticks_diff(time.ticks_ms(), read_start)
            
            decode_start = time.ticks_ms()
            decode_acceleration(sensor, accel_buffer, accel_array)
            decode_dur = time.ticks_diff(time.ticks_ms(), read_start)
            x,y,z = accel_array[0:3]

            print('fifo-read', read_start/1000, fifo_level, read_dur, decode_dur,
                  '{0:.2f},{1:.2f},{2:.2f}'.format(x, y, z))
            

        #print('set text', text)
        #update_display(display, text)
    
        # Wait
        time.sleep(0.10)

main()
