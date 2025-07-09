import time
class StopWatch():
    def __init__(self):
        self.first_time = None

    def start(self):
        self.first_time = time.time()

    def update(self):
        return time.time() - self.first_time