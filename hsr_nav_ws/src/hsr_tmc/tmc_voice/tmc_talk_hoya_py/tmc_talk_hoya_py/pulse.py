'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
# -*- coding: utf-8 -*-
import ctypes

# https://freedesktop.org/software/pulseaudio/doxygen/simple.html
try:
    _lib = ctypes.CDLL('libpulse-simple.so.0')
except Exception:
    # libpulse0 (1:13.99.1-1ubuntu3.13) install to /usr/lib/x86_64-linux-gnu/
    _lib = ctypes.CDLL('/usr/lib/x86_64-linux-gnu/libpulse-simple.so.0')

PA_SAMPLE_U8 = 0
PA_SAMPLE_ALAW = 1
PA_SAMPLE_ULAW = 2
PA_SAMPLE_S16LE = 3
PA_SAMPLE_S16BE = 4
PA_SAMPLE_FLOAT32LE = 5
PA_SAMPLE_FLOAT32BE = 6
PA_SAMPLE_S32LE = 7
PA_SAMPLE_S32BE = 8
PA_SAMPLE_S24LE = 9
PA_SAMPLE_S24BE = 10
PA_SAMPLE_S24_32LE = 11
PA_SAMPLE_S24_32BE = 12
PA_SAMPLE_MAX = 13
PA_SAMPLE_INVALID = -1

PA_STREAM_NODIRECTION = 0
PA_STREAM_PLAYBACK = 1
PA_STREAM_RECORD = 2
PA_STREAM_UPLOAD = 3

STRING = ctypes.c_char_p
pa_channel_position_t = ctypes.c_int
pa_sample_format_t = ctypes.c_int
pa_stream_direction_t = ctypes.c_int


class pa_simple(ctypes.Structure):
    _fields_ = [
        ('mainloop', ctypes.c_void_p),
        ('context', ctypes.c_void_p),
        ('stream', ctypes.c_void_p),
        ('direction', pa_stream_direction_t),
        ('read_data', ctypes.c_void_p),
        ('read_length', ctypes.c_size_t),
        ('operation_success', ctypes.c_int)
    ]


class pa_channel_map(ctypes.Structure):
    _fields_ = [
        ('channels', ctypes.c_uint8),
        ('map', pa_channel_position_t * 32),
    ]


class pa_sample_spec(ctypes.Structure):
    _fields_ = [
        ('format', pa_sample_format_t),
        ('rate', ctypes.c_uint32),
        ('channels', ctypes.c_uint8),
    ]


class pa_buffer_attr(ctypes.Structure):
    _fields_ = [
        ('maxlength', ctypes.c_uint32),
        ('tlength', ctypes.c_uint32),
        ('prebuf', ctypes.c_uint32),
        ('minreq', ctypes.c_uint32),
        ('fragsize', ctypes.c_uint32),
    ]


pa_simple_new = _lib.pa_simple_new
pa_simple_new.restype = ctypes.POINTER(pa_simple)
pa_simple_new.argtypes = [
    STRING,
    STRING,
    pa_stream_direction_t,
    STRING,
    STRING,
    ctypes.POINTER(pa_sample_spec),
    ctypes.POINTER(pa_channel_map),
    ctypes.POINTER(pa_buffer_attr),
    ctypes.POINTER(ctypes.c_int)
]
pa_simple_free = _lib.pa_simple_free
pa_simple_free.restype = None
pa_simple_free.argtypes = [
    ctypes.POINTER(pa_simple)
]

pa_simple_write = _lib.pa_simple_write
pa_simple_write.restype = ctypes.c_int
pa_simple_write.argtypes = [
    ctypes.POINTER(pa_simple),
    ctypes.POINTER(ctypes.c_byte),
    ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_int)
]
