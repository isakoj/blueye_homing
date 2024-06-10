import threading
import time

class WatchdogTimer:
    def __init__(self, timeout, user_handler=None):
        """
        Initialize the WatchdogTimer.
        :param timeout: Timeout in seconds after which the watchdog triggers.
        :param user_handler: User-defined function to call when the timeout expires.
        """
        self.timeout = timeout
        self.handler = user_handler if user_handler else self.default_handler
        self.timer = None

    def start(self):
        """
        Start the watchdog timer.
        """
        self.stop()  # Ensure any existing timer is stopped
        self.timer = threading.Timer(self.timeout, self.handler)
        self.timer.start()

    def stop(self):
        """
        Stop the watchdog timer.
        """
        if self.timer:
            self.timer.cancel()
            self.timer = None

    def reset(self):
        """
        Reset the watchdog timer.
        """
        self.start()

    def default_handler(self):
        """
        Default handler that is called when the watchdog timer expires.
        """
        print("Watchdog timer expired!")
