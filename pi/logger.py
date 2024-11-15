import time
import json

import os, pty


class Logger:
    def __init__(self, file_path):
        self.file_path = file_path
        self.file = open(self.file_path, 'w')

    def log_new_data(self, data):
        json.dump(data, self.file, indent=4)
        self.file.flush()
