#!/usr/bin/env python3

import speech_recognition as sr

# obtain audio from the microphone
with sr.Microphone() as source:
    r = sr.Recognizer()
    print("Say something!")
    #r.operation_timeout = 5
    audio = r.listen(source, timeout=5, phrase_time_limit=5)

# recognize speech using Google Cloud Speech
# GOOGLE_CLOUD_SPEECH_CREDENTIALS = r"""INSERT THE CONTENTS OF THE GOOGLE CLOUD SPEECH JSON CREDENTIALS FILE HERE"""
try:
    print("Google Cloud Speech thinks you said " + r.recognize_google_cloud(audio))
except sr.UnknownValueError:
    print("Google Cloud Speech could not understand audio")
except sr.RequestError as e:
    print("Could not request results from Google Cloud Speech service; {0}".format(e))