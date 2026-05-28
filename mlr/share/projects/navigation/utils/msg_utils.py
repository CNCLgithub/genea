import sys


class Msg:
    ERROR = "ERR"
    SUCCESS = "SUCCESS"

    IS_DEBUG = False

    @staticmethod
    def print_error(message):
        sys.stderr.write('\x1b[1;31m' + message.strip() + '\x1b[0m\n')  # bold, red

    @staticmethod
    def print_error_thin(message):
        sys.stderr.write('\x1b[0;31m' + message.strip() + '\x1b[0m\n')  # non-bold, red

    @staticmethod
    def print_success(message):
        sys.stdout.write('\x1b[1;32m' + message.strip() + '\x1b[0m\n')  # bold, green

    @staticmethod
    def print_success_thin(message):
        sys.stdout.write('\x1b[0;32m' + message.strip() + '\x1b[0m\n')  # non-bold, green

    @staticmethod
    def print_warn(message):
        sys.stderr.write('\x1b[1;33m' + message.strip() + '\x1b[0m\n')  # bold, yellow

    @staticmethod
    def print_info(message):
        if Msg.IS_DEBUG:
            sys.stdout.write('\x1b[1;34m' + message.strip() + '\x1b[0m\n')  # bold, blue

    @staticmethod
    def print_normal(message):
        sys.stdout.write(message.strip())

    @staticmethod
    def println_normal(message):
        sys.stdout.write(message.strip() + '\n')
        sys.stdout.flush()
