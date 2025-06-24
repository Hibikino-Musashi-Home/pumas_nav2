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
#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time
import unittest

from unittest.mock import patch

from tmc_talk_hoya_py import (
    VoiceText,
    VoiceTextLibvtNotFound,
    VoiceTextLicenseNotFound,
    VoiceTextRuntimeError,
    VoiceTextSpeaker
)


class TextToBuffer(object):
    u"""VoiceText Mock of TextTobuffer

    See VoiceText JAPANESE ENGINE API PROGRAMMER'S GUIDE for details
    """

    def __init__(self):
        self._count = 1

    def __call__(self,
                 fmt,
                 tts_text,
                 output_buff,
                 output_len,
                 flag,
                 nThreadID,
                 nSpeakerID,
                 pitch,
                 speed,
                 volume,
                 pause,
                 dictidx,
                 texttype):
        # Always set 0.1 seconds
        output_len._obj.value = 3200
        if tts_text is None:
            return 0
        # If a blank character string, returns -4
        if len(tts_text) == 0:
            return -4
        # Return -5 (error) for "error"
        if tts_text == "error":
            return -5
        # Return 1 at the end if the length of the string
        if self._count >= len(tts_text):
            return 1
        self._count += 1
        return 0


class TestVoiceText(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.test_path = os.path.join(os.path.dirname(__file__), 'license')

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_to_file_fails(self, ao, vtlib):
        u"""If an error occurs in Texttofile, an exception can be applied"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToFile.side_effect = [-1]
        vt = VoiceText(path=self.test_path, voice='bridget')
        self.assertRaises(VoiceTextRuntimeError,
                          lambda: vt.to_file(u"test", "/tmp/test.wave"))

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_to_file(self, ao, vtlib):
        u"""Test if you do not have an error"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToFile.side_effect = [1]
        vt = VoiceText(path=self.test_path, voice='bridget')
        self.assertTrue(vt.to_file(u"test", "/tmp/test.wave"))

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_to_file_with_empty_string(self, ao, vtlib):
        u"""Testing whether to return FALSE without an error even in the empty string (does it get an error even if VoiceText returns -4)"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToFile.side_effect = [-4]
        vt = VoiceText(path=self.test_path, voice='bridget')
        self.assertFalse(vt.to_file(u"test", "/tmp/test.wave"))


class TestVoiceTextSpeaker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.test_path = os.path.join(os.path.dirname(__file__), 'license')

    def test_japanese_license(self):
        u"""Confirmation of Japanese license"""
        self.assertRaises(VoiceTextLicenseNotFound,
                          lambda: VoiceTextSpeaker(path=self.test_path,
                                                   voice='test'))
        self.assertRaises(VoiceTextLicenseNotFound,
                          lambda: VoiceTextSpeaker(path=self.test_path,
                                                   voice='hanako'))
        self.assertRaises(VoiceTextLibvtNotFound,
                          lambda: VoiceTextSpeaker(path=self.test_path,
                                                   voice='haruka'))

    def test_english_license_not_exist(self):
        u"""Confirmation of English license"""
        self.assertRaises(VoiceTextLicenseNotFound,
                          lambda: VoiceTextSpeaker(path=self.test_path,
                                                   voice='test'))
        self.assertRaises(VoiceTextLicenseNotFound,
                          lambda: VoiceTextSpeaker(path=self.test_path,
                                                   voice='mark'))
        self.assertRaises(VoiceTextLibvtNotFound,
                          lambda: VoiceTextSpeaker(path=self.test_path,
                                                   voice='julie'))

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_init_fail(self, ao, vtlib):
        u"""Failed to initialize VoiceText"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = -1
        self.assertRaises(VoiceTextRuntimeError,
                          lambda: VoiceTextSpeaker(path=self.test_path,
                                                   voice='bridget'))

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_speak_in_english(self, ao, vtlib):
        u"""Will VoiceText return the end time when VoiceText behaves in the English language version?"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToBuffer.side_effect = TextToBuffer()
        with VoiceTextSpeaker(path=self.test_path, voice='bridget') as speaker:
            self.assertAlmostEqual(speaker.speak(u"123"), 0.3)

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_speak_in_japanese(self, ao, vtlib):
        u"""Do you return the end time when VoiceText behaves in the Japanese version?"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToBuffer.side_effect = TextToBuffer()
        instance.language = "jpn"
        with VoiceTextSpeaker(path=self.test_path, voice='sakura') as speaker:
            self.assertAlmostEqual(speaker.speak(u"日本語"), 0.6)

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_speak_encoding_error_in_english(self, ao, vtlib):
        u"""Will UnicodingEncodeRror occur if I speak Japanese in the English version?"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToBuffer.side_effect = TextToBuffer()
        with VoiceTextSpeaker(path=self.test_path, voice='bridget') as speaker:
            self.assertRaises(UnicodeEncodeError,
                              lambda: speaker.speak(u"日本語"))

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_speak_encoding_error_in_japanese(self, ao, vtlib):
        u"""Will UnicodingEncodeRror generate if you speak the characters that cause encoding errors in the Japanese version?"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToBuffer.side_effect = TextToBuffer()
        instance.language = "jpn"
        with VoiceTextSpeaker(path=self.test_path, voice='sakura') as speaker:
            self.assertRaises(UnicodeEncodeError,
                              lambda: speaker.speak(u"\u4039"))

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_speak_with_emptry_string(self, ao, vtlib):
        u"""Testing whether an error occurs even in the empty string (does it get an error even if VoiceText returns -4)"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToBuffer.side_effect = TextToBuffer()
        with VoiceTextSpeaker(path=self.test_path, voice='bridget') as speaker:
            self.assertAlmostEqual(speaker.speak(u""), 0.0)

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_speak_voicetext_fails(self, ao, vtlib):
        u"""If an error occurs in VOICETEXT, an exception can be applied"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToBuffer.side_effect = TextToBuffer()
        with VoiceTextSpeaker(path=self.test_path, voice='bridget') as speaker:
            self.assertRaises(VoiceTextRuntimeError,
                              lambda: speaker.speak(u"error"))

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_cancel_on_talking(self, ao, vtlib):
        u"""Can I stop writing to pulseAudio with cancellation during the speech?"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToBuffer.side_effect = TextToBuffer()
        with VoiceTextSpeaker(path=self.test_path, voice='bridget') as speaker:
            speaker.speak(u"1234")
            time.sleep(0.25)
            speaker.cancel()
            self.assertAlmostEqual(ao.write.call_count, 3)

    @patch('tmc_talk_hoya_py.voicetext.VoiceTextLibrary')
    @patch('tmc_talk_hoya_py.voicetext.AudioOut')
    def test_speak_on_talking(self, ao, vtlib):
        u"""Is the utterance in the middle of the utterance canceled?"""
        instance = vtlib.return_value
        instance.VT_LOADTTS.return_value = 0
        instance.VT_TextToBuffer.side_effect = TextToBuffer()
        with VoiceTextSpeaker(path=self.test_path, voice='bridget') as speaker:
            speaker.speak(u"123")
            time.sleep(0.05)
            speaker.speak(u"123")
            time.sleep(0.3)
            self.assertAlmostEqual(ao.write.call_count, 4)
