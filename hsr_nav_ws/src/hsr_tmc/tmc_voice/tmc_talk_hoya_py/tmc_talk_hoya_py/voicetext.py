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
import copy
import ctypes
import glob
import os

import queue as Queue
import threading

import tmc_talk_hoya_py.pulse as pulse


class VoiceTextRuntimeError(RuntimeError):
    pass


class VoiceTextLicenseNotFound(VoiceTextRuntimeError):
    pass


class VoiceTextLibvtNotFound(VoiceTextRuntimeError):
    pass


class VoiceTextLibrary(object):
    u"""Cut out so that MOCK testing is easy"""

    def __init__(self, library):
        self._lang = os.path.basename(library).split('_')[1][:-3]
        self._lib = ctypes.cdll.LoadLibrary(library)
        self.VT_LOADTTS = getattr(
            self._lib, 'VT_LOADTTS_{0}'.format(self._lang.upper()))
        self.VT_TextToBuffer = getattr(
            self._lib, 'VT_TextToBuffer_{0}'.format(self._lang.upper()))
        self.VT_TextToFile = getattr(
            self._lib, 'VT_TextToFile_{0}'.format(self._lang.upper()))

    @property
    def language(self):
        return self._lang


class AudioOut(object):
    u"""Cut out so that MOCK testing is easy"""

    def __init__(self):
        ss = pulse.pa_sample_spec()
        ss.format = pulse.PA_SAMPLE_S16LE
        ss.channels = 1
        ss.rate = 16000
        name = b"VoiceTextSpeaker"
        stream_name = b"Voice"
        self._pulse = pulse.pa_simple_new(
            None,                # Use the default server.
            name,                # Our application's name.
            pulse.PA_STREAM_PLAYBACK,
            None,                # Use the default device.
            stream_name,         # Description of our stream.
            ss,                  # Our sample format.
            None,                # Use default channel map
            None,                # Use default buffering attributes.
            None,                # Ignore error code.
        )

    def write(self, buf):
        pulse.pa_simple_write(self._pulse, buf, len(buf), None)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pulse.pa_simple_free(self._pulse)
        return True


class VoiceText(object):
    VT_BUFFER_API_FMT_S16PCM = 0
    VT_FILE_API_FMT_S16PCM = 0       # 16bits Linear PCM
    VT_FILE_API_FMT_ALAW = 1         # 8bits A-law PCM
    VT_FILE_API_FMT_MULAW = 2        # 8bits Mu-law PCM
    VT_FILE_API_FMT_DADPCM = 3       # 4bits Dialogic ADPCM
    VT_FILE_API_FMT_S16PCM_WAVE = 4  # 16bits Linear PCM WAVE
    VT_FILE_API_FMT_U08PCM_WAVE = 5  # 8bits Unsigned Linear PCM WAVE
    # VT_FILE_API_FMT_IMA_WAVE = 6    (not supported)
    VT_FILE_API_FMT_ALAW_WAVE = 7    # 8bits A-law PCM WAVE
    VT_FILE_API_FMT_MULAW_WAVE = 8   # 8bits Mu-law PCM WAVE
    VT_FILE_API_FMT_MULAW_AU = 9     # 8bits Mu-law PCM SUN AU

    def __init__(self, path='/opt/tmc/vt', voice='haruka', iotype='RAMIO'):
        root_path = os.path.join(path, voice, 'M16')
        license_path = root_path + '/data-common/verify/verification.txt'
        if not os.path.exists(license_path):
            raise VoiceTextLicenseNotFound(
                "Voice text license " + license_path + " is not found.")
        libs = glob.glob(os.path.join(
            root_path, 'bin', iotype, 'LINUX64_GLIBC3', 'libvt_*.so'))
        if not libs or len(libs) > 1:
            raise VoiceTextLibvtNotFound(
                "Proper voice text library for " + voice + " is not found. :" + str(libs))
        lib_path = libs[0]
        self._libvt = VoiceTextLibrary(lib_path)
        ret = self._libvt.VT_LOADTTS(None, -1, root_path.encode(), None)
        if not ret == 0:
            raise VoiceTextRuntimeError("Failed to initialize VoiceText")
        slen = ctypes.c_int(0)
        ret = self._libvt.VT_TextToBuffer(
            self.VT_BUFFER_API_FMT_S16PCM,
            None,
            None,
            ctypes.byref(slen),
            -1, 0, -1, -1, -1, -1, -1, -1, -1)
        self._buf = (ctypes.c_byte * slen.value)()

    def encode_message(self, msg):
        if self._libvt.language == 'jpn':
            return msg.encode('cp932')
        else:
            return msg.encode('cp1252')

    def to_buffer(self, msg, pitch=-1, speed=-1, volume=-1, pause=-1):
        slen = ctypes.c_int(0)
        flag = 0
        while True:
            # If you can't encode, UNICODEENCODEERROR is thrown here
            ret = self._libvt.VT_TextToBuffer(
                self.VT_BUFFER_API_FMT_S16PCM,
                self.encode_message(msg),
                self._buf,
                ctypes.byref(slen),
                flag, 0, -1, pitch, speed, volume, pause, -1, -1)
            flag = 1
            if ret >= 0:
                yield ((ctypes.c_byte * slen.value).from_address(
                    ctypes.addressof(self._buf)), slen.value / 32000.0)
                if ret == 1:
                    break
            elif ret == -4:  # When the character length is 0
                break
            else:
                # [-1] When using the non-supported audio format
                # [-2] If you fail to secure a channel memory
                # [-3] When the text character string is NULL POINTER
                # [-4] When the length of the text character string is 0
                # [-5] If the frame buffer is NULL POINTER
                # [-6] When the composite DB of the speaker is not loaded
                # [-7] When the corresponding Thread ID is already in use
                # [-8] If an error that has an unknown reason occurs
                raise VoiceTextRuntimeError(
                    "VT_TextToBuffer failed. ret={0}".format(ret))

    def to_file(self, msg, filename,
                format=VT_FILE_API_FMT_S16PCM,
                pitch=-1, speed=-1, volume=-1, pause=-1):
        filename = filename.encode()
        ret = self._libvt.VT_TextToFile(
            format,
            self.encode_message(msg),
            filename,
            -1, pitch, speed, volume, pause, 0, 0)
        if ret == 1:
            return True
        elif ret == -4:
            return False
        else:
            # [-1] When using the non-supported audio format
            # [-2] If you fail to secure a channel memory
            # [-3] When the text character string is NULL POINTER
            # [-4] When the length of the text character string is 0
            # [-5] When the composite DB of the speaker is not loaded
            # [-6] If you fail to generate an audio file
            # [-7] Other reasons
            raise VoiceTextRuntimeError(
                "VT_TextToFile failed. ret={0}".format(ret))


class VoiceTextSpeaker(object):
    def __init__(self, path='/opt/tmc/vt', voice='haruka', iotype='RAMIO'):
        self._vt_lib = VoiceText(path, voice=voice, iotype=iotype)
        self._audio_out = AudioOut()
        self._queue = Queue.Queue()
        self._thread = threading.Thread(target=self._write)
        self._thread.setDaemon(True)
        self._finish = False
        self._thread.start()

    def __enter__(self):
        self._audio_out.__enter__()
        return self

    def __exit__(self, *args):
        self._finish = True
        self._queue.put((None, None))
        self._thread.join()
        self._audio_out.__exit__()
        return True

    def speak(self, msg, pitch=-1, speed=-1, volume=-1, pause=-1):
        total = 0.0
        for buf, duration in self._vt_lib.to_buffer(msg,
                                                    pitch=pitch,
                                                    speed=speed,
                                                    volume=volume,
                                                    pause=pause):
            self._queue.put((copy.deepcopy(buf), duration), False)
            total = total + duration
        return total

    def cancel(self):
        while not self._queue.empty():
            self._queue.get(False)

    def _write(self):
        while True:
            buf, duration = self._queue.get(True)
            if self._finish:
                break
            self._audio_out.write(buf)
