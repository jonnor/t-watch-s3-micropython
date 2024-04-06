
import os

def isoformat(dt):
    # YEAR-MONTH-DAY T HOUR:MINUTE:SECOND
    # index nr 3 is weekday, ignored
    s = "%d-%02d-%02dT%02d:%02d:%02d" % (dt[0], dt[1], dt[2], dt[4], dt[5], dt[6])
    return s

def dir_exists(filename):
    try:
        return (os.stat(filename)[0] & 0x4000) != 0
    except OSError:
        return False

