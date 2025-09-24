import struct
import serial
import plotly.graph_objects as go
from typing import List
from prompt_toolkit.validation import ValidationError


class PhysicalLayer(object):
    def __init__(self, port: str, baud: int):
        self.port = serial.Serial(port, baud)
        self.port.reset_input_buffer()

    def write(self, data):
        self.port.write(data)
        self.port.flushOutput()

    def read(self, num: int):
        return self.port.read(num)


class Protocol(object):
    def __init__(self, phy: PhysicalLayer):
        self.commands = {
            'cycle': struct.pack('>B', 0),
            'set_ascent_rate': struct.pack('>B', 1),
            'set_descent_rate': struct.pack('>B', 2),
            'set_timeout': struct.pack('>B', 3),
            'set_ramp_rate': struct.pack('>B', 4),
            'get_ascent_rate': struct.pack('>B', 5),
            'get_descent_rate': struct.pack('>B', 6),
            'get_timeout': struct.pack('>B', 7),
            'get_ramp_rate': struct.pack('>B', 8),
            'get_previous_level': struct.pack('>B', 9),
            'send_motor_up':struct.pack('>B', 10),
            'send_motor_down':struct.pack('>B', 11),
            'send_motor_stop': struct.pack('>B', 12)
        }
        self.variables = {'uint8': {'packing': '>B', 'length': 1}, 'uint16': {'packing': '>H', 'length': 2},
                          'uint32': {'packing': '>I', 'length': 4}, 'int8': {'packing': '>b', 'length': 1},
                          'int16': {'packing': '>h', 'length': 2}, 'int32': {'packing': '>i', 'length': 4},
                          'bool': {'packing': '>B', 'length': 1}}

        self.phy = phy

    def execute_cycle(self):
        self.phy.write(self.commands['cycle'])
        print('cycle sent')
        return_data = []
        while True:
            val = struct.unpack('>H', self.phy.read(2))[0]
            if val != 0xFFFF:
                return_data.append(val)
            else:
                break

        fig = go.Figure()
        fig.add_trace(go.Scatter(y=return_data))
        fig.show()

    def execute_set_ascent_rate(self, val: int):
        data = self.commands['set_ascent_rate'] + struct.pack(self.variables['uint8']['packing'], val)
        self.phy.write(data)
        print('set_ascent_rate sent')

    def execute_set_descent_rate(self, val: int):
        data = self.commands['set_descent_rate'] + struct.pack(self.variables['uint8']['packing'], val)
        self.phy.write(data)
        print('set_descent_rate sent')

    def execute_set_timeout(self, val: List[int]):
        data = self.commands['set_timeout'] + struct.pack(self.variables['uint8']['packing'], val[0]) + \
               struct.pack(self.variables['uint16']['packing'], val[1])
        self.phy.write(data)
        print('set_timeout sent')

    def execute_set_ramp_rate(self, val: int):
        data = self.commands['set_ramp_rate'] + struct.pack(self.variables['uint16']['packing'], val)
        self.phy.write(data)
        print('set_ramp_rate sent')

    def execute_get_ascent_rate(self):
        self.phy.write(self.commands['get_ascent_rate'])
        print('get_ascent_rate sent')
        return_data = self.phy.read(1 * self.variables['uint8']['length'])
        print([x[0] for x in struct.iter_unpack(self.variables['uint8']['packing'], return_data)])

    def execute_get_descent_rate(self):
        self.phy.write(self.commands['get_descent_rate'])
        print('get_descent_rate sent')
        return_data = self.phy.read(1 * self.variables['uint8']['length'])
        print([x[0] for x in struct.iter_unpack(self.variables['uint8']['packing'], return_data)])

    def execute_get_timeout(self, val: int):
        data = self.commands['get_timeout'] + struct.pack(self.variables['uint8']['packing'], val)
        self.phy.write(data)
        print('get_timeout sent')
        return_data = self.phy.read(1 * self.variables['uint16']['length'])
        print([x[0] for x in struct.iter_unpack(self.variables['uint16']['packing'], return_data)])

    def execute_get_ramp_rate(self):
        self.phy.write(self.commands['get_ramp_rate'])
        print('get_ramp_rate sent')
        return_data = self.phy.read(1 * self.variables['uint16']['length'])
        print([x[0] for x in struct.iter_unpack(self.variables['uint16']['packing'], return_data)])

    def execute_get_previous_level(self):
        self.phy.write(self.commands['get_previous_level'])
        print('get_previous_level sent')
        return_data = self.phy.read(2 * self.variables['uint16']['length'])
        print([x[0] for x in struct.iter_unpack(self.variables['uint16']['packing'], return_data)])
        
    def send_motor_up(self):
        self.phy.write(self.commands['send_motor_up'])

    def send_motor_down(self):
        self.phy.write(self.commands['send_motor_down'])

    def send_motor_stop(self):
        self.phy.write(self.commands['send_motor_stop'])
        
    def execute_command(self, text: str):
        """ Executes a command. Not this will not validate, recommended that command is tested first with
        validate_command

        Args:
            text:
        """
        command = text.split(' ')[0]
        if command == 'cycle':
            return self.execute_cycle()
        elif command == 'set_ascent_rate':
            return self.execute_set_ascent_rate(self.validate_set_ascent_rate(text))
        elif command == 'set_descent_rate':
            return self.execute_set_descent_rate(self.validate_set_descent_rate(text))
        elif command == 'set_timeout':
            return self.execute_set_timeout(self.validate_set_timeout(text))
        elif command == 'set_ramp_rate':
            return self.execute_set_ramp_rate(self.validate_set_ramp_rate(text))
        elif command == 'get_ascent_rate':
            return self.execute_get_ascent_rate()
        elif command == 'get_descent_rate':
            return self.execute_get_descent_rate()
        elif command == 'get_timeout':
            return self.execute_get_timeout(self.validate_get_timeout(text))
        elif command == 'get_ramp_rate':
            return self.execute_get_ramp_rate()
        elif command == 'get_previous_level':
            return self.execute_get_previous_level()
        elif command == 'send_motor_up':
            return self.execute_send_motor_up()
        elif command == 'send_motor_down':
            return self.execute_send_motor_down()
        elif command == 'send_motor_stop':
            return self.execute_send_motor_stop()   

    @staticmethod
    def validate_cycle(text):
        if text != 'cycle':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=5)

    @staticmethod
    def validate_set_ascent_rate(text):
        try:
            args = text.split(' ')
            val = int(args[1], 0)
            if val < 0 or val > 100:
                raise ValidationError(message='Argument must be within [0, 100]', cursor_position=15)
            return val
        except IndexError:
            raise ValidationError(message='Provide a uint8 argument', cursor_position=15)
        except ValueError:
            raise ValidationError(message='Provide a valid uint8 argument', cursor_position=15)

    @staticmethod
    def validate_set_descent_rate(text):
        try:
            args = text.split(' ')
            val = int(args[1], 0)
            if val < 0 or val > 100:
                raise ValidationError(message='Argument must be within [0, 100]', cursor_position=16)
            return val
        except IndexError:
            raise ValidationError(message='Provide a uint8 argument', cursor_position=16)
        except ValueError:
            raise ValidationError(message='Provide a valid uint8 argument', cursor_position=16)

    @staticmethod
    def validate_set_timeout(text):
        try:
            args = text.split(' ')
            if(len(args) - 1) != 2:
                raise ValidationError(message='Provide 2 arguments', cursor_position=11)
            return_data = []
            for arg in args[1:]:
                val = int(arg, 0)
                if val < 0 or val > 65535:
                    raise ValidationError(message='Argument must be within [0, 65535]', cursor_position=11)
                return_data.append(val)
            return return_data
        except IndexError:
            raise ValidationError(message='Provide a uint16 argument', cursor_position=11)
        except ValueError:
            raise ValidationError(message='Provide a valid uint16 argument', cursor_position=11)

    @staticmethod
    def validate_set_ramp_rate(text):
        try:
            args = text.split(' ')
            val = int(args[1], 0)
            if val < 0 or val > 65535:
                raise ValidationError(message='Argument must be within [0, 65535]', cursor_position=13)
            return val
        except IndexError:
            raise ValidationError(message='Provide a uint16 argument', cursor_position=13)
        except ValueError:
            raise ValidationError(message='Provide a valid uint16 argument', cursor_position=13)

    @staticmethod
    def validate_get_ascent_rate(text):
        if text != 'get_ascent_rate':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=15)

    @staticmethod
    def validate_get_descent_rate(text):
        if text != 'get_descent_rate':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=16)

    @staticmethod
    def validate_get_timeout(text):
        try:
            args = text.split(' ')
            val = int(args[1], 0)
            if val < 0 or val > 3:
                raise ValidationError(message='Argument must be within [0, 3]', cursor_position=11)
            return val
        except IndexError:
            raise ValidationError(message='Provide a uint8 argument', cursor_position=11)
        except ValueError:
            raise ValidationError(message='Provide a valid uint8 argument', cursor_position=11)

    @staticmethod
    def validate_get_ramp_rate(text):
        if text != 'get_ramp_rate':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=13)

    @staticmethod
    def validate_get_previous_level(text):
        if text != 'get_previous_level':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=18)

    @staticmethod
    def validate_send_motor_up(text):
        if text != 'send_motor_up':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=18)

    @staticmethod
    def validate_send_motor_down(text):
        if text != 'send_motor_down':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=18)

    @staticmethod
    def validate_send_motor_stop(text):
        if text != 'send_motor_stop':
            raise ValidationError(message='The input contains unexpected characters', cursor_position=18)

    @staticmethod
    def validate_command(text: str):
        """ Determines in text begins with a valid command

        Args:
            text:
        """
        command = text.split(' ')[0]
        if command == 'cycle':
            return Protocol.validate_cycle
        elif command == 'set_ascent_rate':
            return Protocol.validate_set_ascent_rate
        elif command == 'set_descent_rate':
            return Protocol.validate_set_descent_rate
        elif command == 'set_timeout':
            return Protocol.validate_set_timeout
        elif command == 'set_ramp_rate':
            return Protocol.validate_set_ramp_rate
        elif command == 'get_ascent_rate':
            return Protocol.validate_get_ascent_rate
        elif command == 'get_descent_rate':
            return Protocol.validate_get_descent_rate
        elif command == 'get_timeout':
            return Protocol.validate_get_timeout
        elif command == 'get_ramp_rate':
            return Protocol.validate_get_ramp_rate
        elif command == 'get_previous_level':
            return Protocol.validate_get_previous_level
        elif command == 'send_motor_up':
            return Protocol.validate_send_motor_up
        elif command == 'send_motor_down':
            return Protocol.validate_send_motor_down
        elif command == 'send_motor_stop':
            return Protocol.validate_send_motor_stop
        
