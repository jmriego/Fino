import struct

from enum import Enum


class Order(Enum):
    """
    Pre-defined orders
    """
    HELLO = 0
    ALREADY_CONNECTED = 1
    RECEIVED = 2
    FORCES = 3
    POSITION = 4
    ERROR = 5
    LOG = 6
    VERSION = 7

def read_order(f):
    """
    :param f: file handler or serial file
    :return: (Order Enum Object)
    """
    return Order(read_i8(f))

def read_i8(f):
    """
    :param f: file handler or serial file
    :return: (int8_t)
    """
    return struct.unpack('<b', bytearray(f.read(1)))[0]


def read_i16(f):
    """
    :param f: file handler or serial file
    :return: (int16_t)
    """
    return struct.unpack('<h', bytearray(f.read(2)))[0]


def read_i32(f):
    """
    :param f: file handler or serial file
    :return: (int32_t)
    """
    return struct.unpack('<l', bytearray(f.read(4)))[0]


def write_i8(f, value):
    """
    :param f: file handler or serial file
    :param value: (int8_t)
    """
    if -128 <= value <= 127:
        f.write(struct.pack('<b', value))
    else:
        print("Value error:{}".format(value))


def write_order(f, order):
    """
    :param f: file handler or serial file
    :param order: (Order Enum Object)
    """
    write_i8(f, order.value)


def write_i16(f, value):
    """
    :param f: file handler or serial file
    :param value: (int16_t)
    """
    f.write(struct.pack('<h', value))


def write_i32(f, value):
    """
    :param f: file handler or serial file
    :param value: (int32_t)
    """
    f.write(struct.pack('<l', value))


