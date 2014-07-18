import json
import os
import sys

from sseclient import SSEClient

deviceID = "XXX"
accessToken = "XXX"

messages = SSEClient('https://api.spark.io/v1/devices/' + deviceID + '/events/?access_token=' + accessToken)

for msg in messages:
    print(msg)