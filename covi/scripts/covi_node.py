#!/usr/bin/env python

__author__ = "vicent.girbes@uv.es (Vicent Girbes Juan)"

import rospy
import math
import serial
import threading

import binascii

from dynamic_reconfigure.server import Server
from covi.cfg import CoviBotConfig
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

BAUD_RATES = (  # In bits per second.
    300,
    600,
    1200,
    2400,
    4800,
    9600,
    14400,
    19200,
    28800,
    38400,
    57600,  # Default.
    115200)

MAX_WHEEL_SPEED = 1000  # mm/s
WHEEL_SEPARATION = 395  # mm

SERIAL_TIMEOUT = 2  # Number of seconds to wait for reads. 2 is generous.
START_DELAY = 5  # Time it takes the Roomba/Turtlebot to boot.

BAUDRATE = 115200
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class DriverError(Exception):
  pass
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class SerialCommandInterface(object):

  """A higher-level wrapper around PySerial specifically designed for use with
  Parallax Eddie Propeller Board.

  """

  def __init__(self, tty='/dev/ttyUSB0', baudrate=BAUDRATE):
    self.ser = serial.Serial(
        port=tty,
        baudrate=baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=SERIAL_TIMEOUT
    )
    
    if(self.ser.isOpen() == False):
        self.ser.open()

#    self.wake()
    self.opcodes = {}

    #TODO: kwc all locking code should be outside of the driver. Instead,
    #could place a lock object in Roomba and let people "with" it
    self.lock = threading.RLock()

  def wake(self):
    """wake up robot."""
    print "wake"
#    self.ser.setRTS(0)
#    time.sleep(0.1)
#    self.ser.setRTS(1)
#    time.sleep(0.75)  # Technically it should wake after 500ms.
#    for i in range(3):
#        self.ser.setRTS(0)
#        time.sleep(0.25)
#        self.ser.setRTS(1)
#        time.sleep(0.25) 

  def add_opcodes(self, opcodes):
    """Add available opcodes to the SCI."""
    self.opcodes.update(opcodes)

  def send(self, bytes):
    """send a string of bytes to the robot."""
    with self.lock:
      self.ser.write(bytes)

  #TODO: kwc the locking should be done at a higher level
  def read(self, num_bytes):
    """Read a string of 'num_bytes' bytes from the robot."""
#    logging.debug('Attempting to read %d bytes from SCI port.' % num_bytes)
#    with self.lock:
    data = self.ser.read(num_bytes)
#    logging.debug('Read %d bytes from SCI port.' % len(data))
#    if not data:
#      raise DriverError('Error reading from SCI port. No data.')
#    if len(data) != num_bytes:
#      raise DriverError('Error reading from SCI port. Wrong data length.')
    return data

  def flush_input(self):
    """Flush input buffer, discarding all its contents."""
    logging.debug('Flushing serial input buffer.')
    self.ser.flushInput()
    
  def inWaiting(self):
    """ InWaiting Called """
    logging.debug('Called inWaiting')
    self.ser.inWaiting()
    
  '''
  def __getattr__(self, name):
    """Eddiebots methods for opcodes on the fly.

    Each opcode method sends the opcode optionally followed by a string of
    parameter.

    """
    #TODO: kwc do static initialization instead
    if name in self.opcodes:
      def send_opcode(input_string):
        logging.debug('sending opcode %s.' % name)
        self.send(self.opcodes[name] + ' ' + input_string)
      return send_opcode
    raise AttributeError
  '''
      
  def getSer(self):
    return self.ser
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class Covibot():

  """Represents a Eddiebot robot."""

  def __init__(self):
    """
    @param sensor_class: Sensor class to use for fetching and decoding sensor data.
    """
    logging.basicConfig(filename='eddiebot_driver.log', level=logging.INFO)
    self.tty = None
    self.sci = None
    self.safe = True
    self.start()
    
  def start(self, tty='/dev/ttyUSB0', baudrate=115200):
    self.tty = tty
    self.sci = SerialCommandInterface(tty, baudrate)
    self.sci.add_opcodes(EDDIE_OPCODES)
      
  def control(self):
    """Start the robot's SCI interface and place it in safe or full mode."""
    logging.info('sending control opcodes.')
#    self.passive() Not implemented in Eddie
#    if self.safe:
#      self.sci.safe()
#    else:
#      self.sci.full()
#    time.sleep(0.5)

  def power_low_side_drivers(self, drivers):
    """Enable or disable power to low side drivers.

    'drivers' should be a list of booleans indicating which low side drivers
    should be powered.

    """
    assert len(drivers) == 3, 'Expecting 3 low side driver power settings.'
    byte = 0
    for driver, power in enumerate(drivers):
      byte += (2 ** driver) * int(power)
    # self.sci.low_side_drivers(byte) Not implemented in Eddie

  def set_digital_outputs(self, value):
    """Enable or disable digital outputs."""
    # self.sci.digital_outputs(value) Not implemented in Eddie

  def soft_reset(self):
    """Do a soft reset of the Turtlebot."""
    logging.info('sending soft reset.')
#    self.sci.soft_reset()
#    time.sleep(START_DELAY)
#    self.passive()
    
    
    #------------------- Roomba methods -----------------------------
  def change_baud_rate(self, baud_rate):
    """Sets the baud rate in bits per second (bps) at which SCI commands and
    data are sent according to the baud code sent in the data byte.

    The default baud rate at power up is 57600 bps. (See Serial Port Settings,
    above.) Once the baud rate is changed, it will persist until Roomba is
    power cycled by removing the battery (or until the battery voltage falls
    below the minimum requir''' ed for processor operation). You must wait 100ms
    after sending this command before sending additional commands at the new
    baud rate. The SCI must be in passive, safe, or full mode to accept this
    command. This command puts the SCI in passive mode.

    """
    if baud_rate not in BAUD_RATES:
      raise DriverError('Invalid baud rate specified.')
    self.sci.baud(baud_rate)
    self.sci = SerialCommandInterface(self.tty, baud_rate)

  def passive(self):
    """Put the robot in passive mode."""
    # self.sci.start() Not implemented in Eddie
    print "passive"
#    time.sleep(0.5)

  def direct_drive(self, velocity_left, velocity_right):
#    print("direct_drive(self, velocity_left, velocity_right)")
#    print("velocity_left: " + str(int(velocity_left)))
#    print("velocity_right: " + str(int(velocity_right)))
    # Mask integers to 2 bytes.
    vl = int(velocity_left) & 0xffff
    vr = int(velocity_right) & 0xffff
    
    parameters = binascii.hexlify(struct.pack('>2H', vr, vl))
    
    parameters_output = ''
    
    for i in range(len(parameters)):
             parameters_output += parameters[i]
             if(i == 3):
                 parameters_output += ' '
        
    self.sci.direct_drive(parameters_output + chr(13))
    
  def drive(self, velocity, radius):
    logging.debug("drive(self, velocity, radius)")
    """controls Roomba's drive wheels.

    NOTE(damonkohler): The following specification applies to both the Roomba
    and the Turtlebot.

    The Roomba takes four data bytes, interpreted as two 16-bit signed values
    using two's complement. The first two bytes specify the average velocity
    of the drive wheels in millimeters per second (mm/s), with the high byte
    being sent first. The next two bytes specify the radius in millimeters at
    which Roomba will turn. The longer radii make Roomba drive straighter,
    while the shorter radii make Roomba turn more. The radius is measured from
    the center of the turning circle to the center of Roomba.

    A drive command with a positive velocity and a positive radius makes
    Roomba drive forward while turning toward the left. A negative radius
    makes Roomba turn toward the right. Special cases for the radius make
    Roomba turn in place or drive straight, as specified below. A negative
    velocity makes Roomba drive backward.

    Also see drive_straight and turn_in_place convenience methods.

    """
    # Mask integers to 2 bytes.
    velocity = int(velocity)# & 0xffff TODO:removed the marks to avoid problem
    radius = int(radius)# & 0xffff
    
    # Convert to direct_drive parameter
    velocity_left = ((2000.0 + radius) / 2000.0) * velocity
    velocity_right = ((2000.0 - radius) / 2000.0) * velocity
    
    # STRAIGHT radius set to velocity to drive straight line FW and BW
    if(RADIUS_STRAIGHT == radius):
        velocity_left = velocity
        velocity_right = velocity
    
    self.direct_drive(velocity_left, velocity_right)
    
  def command_joints(self, pan_degree, tilt_degree):
    pan_degree = int(pan_degree)
    tilt_degree = int(tilt_degree)
    parameters = binascii.hexlify(struct.pack('>H', pan_degree))
    self.sci.motors(' 4 ' + parameters + chr(13)) # Pan Servo
    
    #parameters = binascii.hexlify(struct.pack('>H', tilt_degree))
    #self.sci.motors(' 5 ' + parameters + chr(13)) # Pan Servo

  def stop(self):
    """Set velocity and radius to 0 to stop movement."""
    self.direct_drive(0, 0)

  def slow_stop(self, velocity):
    """Slowly reduce the velocity to 0 to stop movement."""
    velocities = xrange(velocity, VELOCITY_SLOW, -8)
    if velocity < 0:
      velocities = xrange(velocity, -VELOCITY_SLOW, 8)
    for v in velocities:
      self.drive(v, RADIUS_STRAIGHT)
      time.sleep(1)
    self.stop()

#  def drive_straight(self, velocity):
#    """drive in a straight line."""
#    self.gospd(velocity)
    
  def drive_straight(self, velocity):
    self.drive(velocity, RADIUS_STRAIGHT)

  def turn_in_place(self, velocity, direction):
    """Turn in place either clockwise or counter-clockwise."""
#    valid_directions = {'cw': RADIUS_TURN_IN_PLACE_CW,
#                        'ccw': RADIUS_TURN_IN_PLACE_CCW}
#    self.drive(velocity, valid_directions[direction])
    velocity = int(velocity)
    if(direction ==  'cw'):
        self.direct_drive(-velocity, velocity)
    if(direction ==  'ccw'):
        self.direct_drive(velocity, -velocity)
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def reconfigure_callback(config, level):
#     self.update_rate = config['update_rate']
#     self.drive_mode = config['drive_mode']
#     self.has_gyro = config['has_gyro']
#  #       if self.has_gyro:
#  #           self._gyro.reconfigure(config, level)
#     self.odom_angular_scale_correction = config['odom_angular_scale_correction']
#     self.odom_linear_scale_correction = config['odom_linear_scale_correction']
#     self.cmd_vel_timeout = rospy.Duration(config['cmd_vel_timeout'])
#     self.stop_motors_on_bump = config['stop_motors_on_bump']
#     self.min_abs_yaw_vel = config['min_abs_yaw_vel']
#     self.max_abs_yaw_vel = config['max_abs_yaw_vel']
    return config
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def covi_main():
    #c = EddiebotNode()
    
    rospy.init_node("covi", anonymous = False)

    srv = Server(CoviBotConfig, reconfigure_callback)
    
    #robot = Covibot()
    sci = SerialCommandInterface(tty='/dev/ttyACM0', baudrate=BAUDRATE)
    
    rate = rospy.Rate(20.0) # 20 Hz
    
    while not rospy.is_shutdown():
#         try:
#             # This sleep throttles reconnecting of the driver.  It
#             # appears that pyserial does not properly release the file
#             # descriptor for the USB port in the event that the Create is
#             # unplugged from the laptop.  This file desecriptor prevents
#             # the create from reassociating with the same USB port when it
#             # is plugged back in.  The solution, for now, is to quickly
#             # exit the driver and let roslaunch respawn the driver until
#             # reconnection occurs.  However, it order to not do bad things
#             # to the Create bootloader, and also to keep relaunching at a
#             # minimum, we have a 3-second sleep.
#             # time.sleep(3.0)
#             
#             c.start()
#             c.spin()
# 
#         except Exception as ex:
#             msg = "Failed to contact device with error: [%s]. Please check that the Create is powered on and that the connector is plugged into the Create."%(ex)
#             c._diagnostics.node_status(msg,"error")
#             rospy.logerr(msg)
# 
#         finally:
#             # Driver no longer connected, delete flag from disk
#             try:
#                 os.remove(connected_file())
#             except Exception: pass
        sci.send('a')
        data = sci.read(1)
        print data
        
        rate.sleep()
        
#     rospy.spin()


if __name__ == '__main__':

    try:
        covi_main()

    except rospy.ROSInterruptException:
        pass
