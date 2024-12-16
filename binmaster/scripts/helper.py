import argparse
from protocol import Protocol, PhysicalLayer
from prompt_toolkit import prompt
from prompt_toolkit.history import FileHistory
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory
from prompt_toolkit.completion import WordCompleter
from prompt_toolkit.validation import Validator, ValidationError


def print_help():
    """
    Print help information
    """
    print('Available commands:\n'
          '\thelp: Output help\n'
          '\tquit: Close prompt\n'
          '\tcycle: Cycle Unit\n'
          '\tset_ascent_rate: Ascent rate (percent)\n'
          '\tset_descent_rate: Descent rate (percent)\n'
          '\tset_timeout: [quadrant] [Timeout (0.1ms)]\n'
          '\tset_ramp_rate: Ramp rate\n'
          '\tget_ascent_rate: Get ascent rate (percent)\n'
          '\tget_descent_rate: Get descent rate (percent)\n'
          '\tget_timeout: Timeout (10us)\n'
          '\tget_ramp_rate: Ramp rate\n'
          '\tget_previous_level: Water level and Bottom level\n')


class ProtocolValidator(Validator):
    @staticmethod
    def validate_help(text):
        if text != 'help':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=3)

    @staticmethod
    def validate_quit(text):
        if text != 'quit':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=3)

    def validate_command(self, text: str):
        """ Determines in text begins with a valid command

        Args:
            text:
        """
        validator = Protocol.validate_command(text)

        if validator is not None:
            return validator
        elif text == 'help':
            return self.validate_help
        elif text == 'quit':
            return self.validate_quit
        else:
            return None

    def validate(self, document):
        text = document.text

        validator = self.validate_command(text)

        if validator is not None:
            validator(text)
        else:
            raise ValidationError(message='Command not found')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-b', '--baud', help='Baudrate', default=1000000, dest='baud', type=int)
    parser.add_argument('-p', '--port', help='Serial Port', default='/dev/ttyACM0', dest='port')
    args = parser.parse_args()

    phy = PhysicalLayer(baud=args.baud, port=args.port)
    mac = Protocol(phy)

    print_help()

    auto_completer = WordCompleter(['cycle', 'set_ascent_rate', 'set_descent_rate', 'set_timeout', 'set_ramp_rate', 'get_ascent_rate', 'get_descent_rate', 'get_timeout', 'get_ramp_rate', 'get_previous_level', 'help', 'quit'],
                                   ignore_case=True)

    while True:
        try:
            user_input = prompt('> ', history=FileHistory('history.txt'),
                                auto_suggest=AutoSuggestFromHistory(),
                                completer=auto_completer,
                                validator=ProtocolValidator())
            if user_input == 'help':
                print_help()
            elif user_input == 'quit':
                break
            else:
                mac.execute_command(user_input)
        except KeyboardInterrupt:
            break
