#!/usr/bin/python

# estimlib.py
# 
# Requires: Python 2.7 + PySerial (pip install pyserial)
#
# MIT License
# 
# GitHub: https://github.com/loopstim
# Email: loopstim [at] outlook [dot] com
# Discussion: https://metafetish.club/c/estim
#
# Copyright (c) 2020 LoopStim
#
# This file can be used as either a library or as a standalone
# script.  In the latter mode, it has a simple command line; the
# [[--help]] option lists a brief summary.

import os
import random
import serial
import threading
import time

#### Usage help

USAGE = """estimlib.py -- an interface to the ErosTek ET-312B

Command line options:

-p, --port=PORTNAME
  The serial communication port to use.  Usually the name of a
  special file such as COM1: on Windows or /dev/ttyS0 on Linux.
  The port can also be specified via the ESTIM_PORT environment
  variable.

-t, --traffic
  Display wire traffic, byte by byte, as it happens.  (This is
  mainly useful for debugging.)

-r, --registers
  Display register loads and stores as they happen.  (This is
  mainly useful for debugging.)

-m, --mask-file=FILENAME
  Store the masking byte to this file upon successful connection
  to the device.  This file is a text file containing a decimal
  number: the masking byte xor 0x55.  This is compatible with
  estim.pm's usage.  The mask file can also be specified via the
  ESTIM_MASK_FILE environment variable.

-i, --interact
  Run an interactive Python shell after initiating connection.
  (This is the default mode.)

-d, --dump
  Dump parts of the device's comm address space to the standard
  output in the standard hex dump form.  The addresses are
  specified as separate command line arguments in the form of
  START-ADDRESS or START-ADDRESS:END-ADDRESS with both addresses
  given in hexadecimal. If END-ADDRESS is missing, it's assumed
  to be 256 bytes after the START-ADDRESS.  If no address ranges
  are specified, the ranges 4000:4200 and 8000:8200 will be
  dumped by default.

-h, --help
  Show this usage help.


Environment variables:

ESTIM_PORT
  The serial communication port name, analogously to the --port
  option.

ESTIM_MASK_FILE
  The name to store the masking byte file in, analogously to the
  --mask-file option.
"""

#### General overview
#
# The ET-312B can be accessed over an RS/232 link via a special
# binary protocol.  The low-level protocol is hardcoded 19200
# baud, 8-bit bytes, no parity check, 1 stop bit (8-N-1).
#


## Wire protocol
#
# Both sides transmit /packets/ in one of two forms: one-byte
# packets consist of a single byte, multibyte packets consist of
# a header byte (which specifies the length of the packet in its
# high nybble), some payload bytes, and a trailing checksum byte
# that is calculated by adding the header byte and the payload
# bytes together mod 256.  Furthermore, bytes transmitted by the
# PC to the device need to be XORed by a /masking byte/, which
# is a state variable of the device.  Upon powerup, the device
# uses [[0x00]] as the masking byte, but the PC is expected to
# negotiate a random masking byte upon successful connection.
# (See [[Estim._handshake]] for the details of guessing the
# masking byte if we don't know it, and [[Estim._negotiateMask]]
# for details on negotiating a new random masking byte.)
#
# There are two main useful command packet types:
# - The [[getByte]] command packet consists of four bytes and
#   has the form [[0x3C hh ll ss]].  Here, [[hhll]] specifies a
#   16-byte address in the device's 'comm address space'
#   (transmitted in the network byte order), and [[ss]]
#   specifies the above-mentioned checksum.  The device responds
#   with [[0x22 vv ss]] where [[vv]] is the value stored in this
#   location of the comm address space.
# - The [[setByte]] command packet consists of five bytes and
#   has the form [[0x4D hh ll vv ss]].  As before, [[hhll]]
#   specifies a 16-byte address, [[vv]] specifies the value to
#   store at this location, and [[ss]] specifies the checksum.
#   Upon success, the device responds with [[06]] ('ACK').
#   Apparently, the device can also respond with [[07]] ('NAK');
#   I'm not sure when this happens, but I'd speculate this may
#   be the case when a request to store into the comm address
#   space's ROM region was made.
#
# In addition, there are two handshaking command packet types:
# - The [[hello]] command packet consists of one byte and has
#   the form [[0x00]].  A normal session begins by the PC
#   transmitting this packet a few times and the device
#   responding with [[0x06]] ('NAK') /every time/.  If the
#   device does not respond every time, it is probably using a
#   non-zero masking byte.
# - The [[negotiateMask]] command packet consists of three bytes
#   and has the form [[0x2F xx ss]] where [[xx]] is an arbitrary
#   byte picked by the PC.  The device responds with [[0x21 yy
#   ss]], and from this point on, the PC is supposed to XOR all
#   transmitted bytes with [[swan(xx) ^ yy ^ 0x55]], the newly
#   established /masking byte/, where [[swan]] is the 'swap
#   nybbles' operator.
#
# If the device is not responding to [[hello]] as expected, the
# manufacturer recommends to power-cycle the machine and try
# again.  However, experiments show that it's viable to just try
# transmitting short series of all 256 possible masking bytes;
# the device should respond with NAK upon receiving each byte.
# (It appears that a false positive is possible; my experimental
# brute forcing code follows up by asking the content of the
# [[routine]] register; if the device responds with a normal
# [[0x22]] message, it's assumed that the masking byte was
# guessed right, and if it responds with NAK or does not
# respond, it's assumed that the masking byte was guessed
# wrong.)  This way, the current masking byte can be
# brute-forced in under two minutes.
#
# CAUTION: the PC should not transmit too rapidly.  The device
# has a known fault mode, apparently triggered by transmitting
# faster than it can process, in which the device is
# transmitting an endless sequence of [[0x06]] ('NAK') bytes and
# does not seem to react to any input from the PC.  The only
# currently known way to recover from this condition is
# power-cycling the device.  The purpose (and purposefulness) of
# this mode is unknown; it may be an anti-tampering measure, or
# it may be a fail-safety measure intended to cause the device
# to stop processing potentially broken incoming requests when
# it can't process them properly.
#
#   (It appears to me that the high nybble of the header byte
#   specifies the packet's size, with [[0x0]] meaning a 1-byte
#   packet and so on, up to [[0xC]] meaning a 13-byte
#   packet.  If the high nybble is greater than [[0xC]], the
#   device NAKs the packet after 13 bytes anyway.
#
#   This hypothesis is supported by [[ET312.java]] containing
#   the method [[ET312.writeMemory(int, byte[])]] that splits
#   the given byte array into parts of no more than 8 bytes each
#   and delivers each part in a single packet, apparently of the
#   form [[0xnD hh ll vv ... ss]] where [[n]] is the number of
#   payload bytes plus 3 (so that for one byte of payload, the
#   header byte is [[0x4D]]) and [[vv ...]] is the payload.)
#


## Memory layout
#
# There are three known potentially interesting areas of memory:
# [[0x0000 .. 0x00FF]], [[0x4000 .. 0x41FF]], and [[0x8000 ..
# 0x81FF]].  It appears that the top two bits of the comm
# address space adresses specify a storage category; one of
# these three could be RAM, one ROM, and one NVRAM/EEPROM.
#
# The operating parameters of channel A are found at addresses
# [[0x4098 .. 0x40BE]], and the corresponding parameters of
# channel B are found at addresses [[0x4198 .. 0x41BE]].  (See
# [[Channel.REGISTERS]].)  This is suggestive of the [[0x4nnn]]
# memory area corresponding to (a part of) the device's RAM.
# The addresses [[0xFD .. 0xFF]] hold the 'box version'
# (perhaps, firmware version?) in the form of
# major.minor.revision, and the address [[0xFC]] holds the 'box
# model' number.  This is suggestive that the area of [[0x0000
# ..  0x00FF]] corresponds to a part of the device's firmware
# ROM.
#
# The highest loaded routine number is stored at [[0x8008]], and
# it seems that the bytecode for persisting user-defined
# routines is also stored in the [[0x8nnn]] memory region.  This
# is suggestive of the [[0x8nnn]] memory region corresponding to
# (a part of) the device's NVRAM or data EEPROM.  (However, the
# vendor's Java app also seems to contain code for storing
# user routines in [[0x4nnn]].  I'd speculate that this is in
# order to support the 'try' feature of saving a user routine in
# the device's volatile memory.)


## Routine selection
#
# The address 0x407B holds the number of the currently selected
# routine.  Unlike the channel registers, writing a new routine
# number into this register does not immediately affect the
# device's operation.  However, the current routine can be
# changed by saving the new routine's number minus one at this
# address, poking [[0x04]] at [[0x4070]], waiting for 18 ms,
# poking [[0x10]] at [[0x4070]], and waiting again for 18 ms.
# It seems to me that saving a value to this address triggers
# some sort of special handling in the device, as it seems to be
# used for similar roles by several methods of the vendor's
# Java app.  The waiting is probably to give the special
# handlers time to run.
#
# CAVEAT: it seems that upon changing routine this way, the
# [[level]] parameter will be initialised at the maximum value,
# which may present safety concerns.  (I do not yet know how and
# whether other channel parameters are affected.) However, the
# device performs a short ramp-up period upon routine change,
# and it may be feasible to adjust the [[level]] parameters for
# both channels during this ramp-up period.
#


## Miscellaneous notes of questionable utility
#
# According to random web pages, it looks likely that the
# microcontroller in the device is an Atmel ATMEGA16.
#
# The firmware upgrade image distributed by the manufacturer is
# just under 16 KiB (specifically, 15872 or 0x3E00 bytes) in
# size.  The device's firmware is upgraded by activating a
# special mode via a button chord on its physical display and
# transmitting the image to the device by the XMODEM protocol.
#
# The address [[0x4018]] seems to hold the current masking byte.
# However, overwriting it does not seem to have any useful
# effect, and the device seems to either ignore the write
# attempt or restore the register's previous value immediately.
#
# The ROM page at [[0x0000..0x00FF]] repeats 64 times, all the
# way up to [[0x3F00..0x3FFF]].  This is consistent with two top
# address lines being used to identify storage type, bottom
# eight to identify specific memory cells, and the intervening 6
# bits being unused.
#
# The NVRAM area at [[0x8000..0x81FF]] repeats 64 times, all the
# way up to [[0xFE00..0xFFFF]].  This is consistent with one top
# address line being used to identify this storage type, bottom
# nine to identify specific memory cells, and the intervening 6
# bits being unused.
#


#### The register descriptors

class Reg (object):
  """Describe a register in the ET-312B address space as having
  the specified offset (from an implicit, context-dependent
  point of origin).  A common register holds a byte-sized
  integer value; the range of the integer may be restricted from
  the bottom end by a given threshold, and (rather rarely) from
  the top end by a given top."""

  _DEFAULT_BEHAVIOUR = object()
  # [[setter]] can be [[None]] if this is a read-only register,
  # an unbound method of the target class (inheriting from
  # [[AbstractRegisterBank]]) if setting this register takes
  # special handling, or [[_DEFAULT_BEHAVIOUR]] to designate the
  # ordinary behaviour.
  def __init__ (this, offset,
      threshold = 0x00,
      top = 0xFF,
      getter = _DEFAULT_BEHAVIOUR,
      setter = _DEFAULT_BEHAVIOUR,
      disabler = None):
    object.__init__(this)
    this.offset = offset
    this.threshold = threshold
    this.top = top
    this.getter = getter
    this.setter = setter
    this.disabler = disabler


  def translateAfterGet (this, value):
    """Convert a byte freshly retrieved from this register into
    user-friendly form.  In the simplest form, this just means
    translating it from the range of [[(threshold .. top)]] to
    the range of [[(0 .. top - threshold)]]."""
    return value - this.threshold


  def upperBound (this):
    return this.top - this.threshold

  def checkRangeBeforeSet (this, value):
    """Check whether a given value (in the user-friendly range
    or form, not the raw range or form) is fit to be saved into
    this register."""
    if not (0 <= value <= this.upperBound()):
      raise ValueError, 'The value %r is not in %i .. %i.' % \
          (value, 0, this.upperBound())


  def translateBeforeSet (this, value):
    """Convert a user-friendly value into an integer suitable
    for saving into this register.  In the simplest form, this
    just means translating it from the range of [[(0 .. top -
    threshold)]] to the range of [[(threshold .. top)]]."""
    this.checkRangeBeforeSet(value)
    return value + this.threshold


class RevReg (Reg):
  """Describe a reverse-mapped register in the ET-312B address
  space as having the specified offset (from an implicit,
  context-dependent point of origin).  A reverse-mapped common
  register holds a byte-sized integer value; the range of the
  integer may be restricted from the bottom end by a given
  threshold; and the meaning of the value space of the register
  is reversed -- so the user-friendly value 0 is stored to the
  device as [[top]] and the highest supported value, [[top -
  threshold]], is stored to the device as [[threshold]]."""


  def translateAfterGet (this, value):
    """Convert a byte freshly retrieved from this register into
    user-friendly form.  For a reverse-mapped register, this
    means translating it from the range of [[(threshold ..
    top)]] to the range of [[(top - threshold .. 0)]]."""
    return this.top - value


  def translateBeforeSet (this, value):
    """Convert a user-friendly value into an integer suitable
    for saving into this register.  For a reverse-mapped
    register, this means translating it from the range of [[(0
    .. top - threshold)]] to the range of [[(top ..
    threshold)]]."""
    this.checkRangeBeforeSet(value)
    return this.top - value


#### Abstract register banks

class AbstractRegisterBank (object):
  # In order for this to work, the class (or, potentially, the
  # instance -- but this would probably not be a very good idea)
  # must define [[REGISTERS]] to map register names too [[Reg]]
  # (or subclass) instances, and the instance must have
  # [[getByte]] and [[setByte]] methods that read bytes from the
  # device or store them to it at the given address relative to
  # this Abstract Register Bank's origin.
  def __getattr__ (this, name):
    try:
      r = this.REGISTERS[name]
    except KeyError:
      raise AttributeError, \
          'The object %r does not have attribute %r.' % \
              (this, name)
    if r.getter is None:
      raise RuntimeError, \
          'Unable to read from write-only register %s.' % name
    elif r.getter is Reg._DEFAULT_BEHAVIOUR:
      return r.translateAfterGet(
          this.getByte(r.offset))
    else:
      # Note that the method is unbound.  Also note that we are
      # not invoking [[r.translateAfterGet]].
      return r.getter(this)


  # In order for [[__setattr__]] to work properly, a class
  # inheriting from [[AbstactRegisterBank]] must define
  # [[KNOWN_ATTR]], a list of known attributes of instances of
  # this class.  [[__setattr__]] will permit stores to these
  # attributes but raise an exception when an attempt is made to
  # set an attribute that appears neither in [[REGISTERS]] nor
  # [[KNOWN_ATTR]].  Note that [[KNOWN_ATTR]] does not inherit
  # properly,  Additionally, [[__setattr__]] needs [[this.lock]]
  # to be defined -- it engages this lock so as to make sure
  # that other threads won't talk to the device between a write
  # operation to the underlying register and the possible
  # preceding query from its guarding option/status register.
  # (It goes without saying that this should be the same lock
  # that lives in the main [[Estim]] instance, not a newly
  # created lock.)  Finally, it needs [[this.ignoreDisablers]]
  # to be defined, but -- unless you're debugging or running the
  # acid test --, it should usually be [[False]].

  # Why do we limit the set of writable attributes this way,
  # which would be rather un-Pythonic in many contexts?  Well,
  # it's a safety hack.  We're using (abusing?) the Python's
  # attribute mechanism in order to implement virtual attributes
  # that are not backed by the Python's object dictionary (nor
  # the [[__slots__]] mechanism) but by registers in an external
  # hardware device.  Ordinarily, Python permits assigning to
  # unknown attributes, and the attributes are just silently
  # created (except when [[__slots__]] in used, of course).  If
  # the user made a typo in an assignment, and we permitted the
  # ordinary Python semantics, the effect would be that an
  # attempt to set a register's value has silently failed (and
  # an attribute was defined to the instance that either won't
  # ever be useful, or, worse, will show up when the user
  # performs a test readback).  In such a context, it's better
  # to raise an exception when an attempt to set the value of an
  # unknown attribute is made, and since the device's design and
  # feature set is completed and is not going to change to any
  # significant degree in the foreseeable future, we can even
  # sacrifice the ordinary inheritability principle of OOP.  (We
  # could do it without this sacrifice by involving the
  # metaobject customisation infrastructure, but that would be
  # overkill.)

  def __setattr__ (this, name, newValue):
    try:
      r = this.REGISTERS[name]
    except KeyError:
      if name not in this.KNOWN_ATTR:
        raise AttributeError, \
            'The object %r does not have attribute %r.' % \
                (this, name)
      # The name refers to an attribute.
      return object.__setattr__(this, name, newValue)

    # The name refers to a register.
    if r.setter is None:
      raise RegisterNotWritable, \
          'Unable to write to read-only register %s.' % name

    with this.lock:
      if (not this.ignoreDisablers) and \
          r.disabler and r.disabler(this):
        raise RegisterCurrentlyDisabled, \
            'Unable to write to disabled register %s.' % name

      elif r.setter is Reg._DEFAULT_BEHAVIOUR:
        this.setByte(r.offset, r.translateBeforeSet(newValue))
      else:
        # This register is so virtual that it is to be set by
        # invoking a method presumedly for additional checks or
        # processing.  Note that the method is unbound.  Also
        # note that we are not invoking
        # [[r.translateBeforeSet]].
        r.setter(this, newValue)


#### The main class to encapsulate an ET312 device

class Estim (AbstractRegisterBank):

  ## Simple response codes from the device
  RES_ACK = 0x06
  RES_NAK = 0x07

  ## Numbers for the routines
  ROUTINE_NONE = 0x00
  ROUTINE_WAVES = 0x76
  ROUTINE_STROKE = 0x77
  ROUTINE_CLIMB = 0x78
  ROUTINE_COMBO = 0x79
  ROUTINE_INTENSE = 0x7A
  ROUTINE_RHYTHM = 0x7B
  ROUTINE_AUDIO1 = 0x7C
  ROUTINE_AUDIO2 = 0x7D
  ROUTINE_AUDIO3 = 0x7E
  ROUTINE_SPLIT = 0x7F
      # The split routine requires special handling -- separate
      # routine setting for each channel.  See
      # [[Estim.setSplitRoutine()]].
  ROUTINE_RANDOM1 = 0x80
  ROUTINE_RANDOM2 = 0x81
  ROUTINE_TOGGLE = 0x80
  ROUTINE_ORGASM = 0x83
  ROUTINE_TORMENT = 0x84
  ROUTINE_PHASE1 = 0x85
  ROUTINE_PHASE2 = 0x86
  ROUTINE_PHASE3 = 0x87
  # The device supports up to seven user-defined modes, but
  # they're only available if the user has loaded them.
  ROUTINE_USER1 = 0x88
  ROUTINE_USER2 = 0x89
  ROUTINE_USER3 = 0x8A
  ROUTINE_USER4 = 0x8B
  ROUTINE_USER5 = 0x8C
  ROUTINE_USER6 = 0x8D
  ROUTINE_USER7 = 0x8E


  def __init__ (this, portName = None,
      saveMask = None,
      showTraffic = False,
      showGets = False,
      showSets = False,
      ignoreDisablers = False,
      timeout = 0.5): # in seconds, float OK
    if portName is None:
      portName = os.environ.get('ESTIM_PORT')
      if portName is None:
        raise RuntimeError, 'portName not specified and the ' \
            'ESTIM_PORT environment variable not set'
    if saveMask is None:
      saveMask = os.environ.get('ESTIM_MASK_FILE')
    object.__init__(this)
    this.port = serial.Serial(port = portName,
      baudrate = 19200,
      bytesize = serial.EIGHTBITS,
      parity = serial.PARITY_NONE,
      stopbits = serial.STOPBITS_ONE,
      timeout = 0.02, # the initial timeout for greeting
      xonxoff = False, # 8-bit clean, no soft flow control
      rtscts = False, dsrdtr = False,
          # there are no wires for hardware flow control
    )
    # First, check for NAK loop.  This is a failure mode --
    # possibly, an anti-tamper mode, possibly, an intended
    # safety mode -- of the device whereby it sends [[RES_NAK]]
    # bytes in rapid succession and does not seem to react to
    # any input.  We can recognise this situation by listening
    # before sending anything: if NAKs come in even though we
    # aren't asking anything, it's probably a NAK loop.
    this.port.flushInput()
    initialInput = this.port.read(3)
    if initialInput == chr(Estim.RES_NAK) * 3:
      raise NakLoop, ('The device is stuck in a NAK loop '
          'and needs to be power-cycled.')

    # Beyond this point, [[this.port]] is only read or written
    # by ask(), which engages the following lock:
    this.lock = threading.RLock()
    # (But note that there are a few other non-read/write
    # accesses to [[this.port]] that must be guarded
    # separately.)  The lock is a reentrant one so that
    # transactions possibly involving multiple sent and received
    # messages could be combined into single atomic units by
    # engaging the same lock.

    this.showTraffic = showTraffic
    this.showGets = showGets
    this.showSets = showSets
    this.ignoreDisablers = ignoreDisablers
    try:
      this._handshake()
    except CommFailure:
      # A communication failure during initialisation indicates
      # initialisation failure.  If we can't talk to the device,
      # we should close our serial port now.
      this.close()
      raise
    # The handshake succeeded.  Let's now set the pySerial port
    # timeout at whatever the caller specified.
    with this.lock:
      this.port.timeout = timeout

    # If we're requested to save the mask byte for use by
    # [[estim.pl]], do it.
    if saveMask is not None:
      with open(saveMask, 'w') as f:
        # For compatibility with [[estim.pl]], we'll save the
        # mask XOR the 0x55 rather than in the plain.
        f.write('%i\n' % (this.mask ^ 0x55))

    # Finally, set up the channel objects.  Well implement a
    # modicum of case insensitivity by binding these to both
    # uppercase and lowercase slots.
    this.A = this.a = Channel(this, 0, ignoreDisablers =
        ignoreDisablers)
    this.B = this.b = Channel(this, 1, ignoreDisablers =
        ignoreDisablers)


  # Let [[__setattr__]] know of our defined attributes:
  KNOWN_ATTR = ('port', 'lock', 'mask',
      'showTraffic', 'showGets', 'showSets', 'ignoreDisablers',
      'A', 'B', 'a', 'b')


  # XXX: Attempting to re-negotiate the mask if it is already
  # non-zero seems to trigger the NAK loop.
  def _negotiateMask (this):
    pcSalt = int(random.random() * 0x100)
    deviceSalt, = this.ask([0x2F, pcSalt], 3, expect = 0x21)
    this.mask = _swapByteNybbles(pcSalt) ^ deviceSalt ^ 0x55


  def _handshake (this):
    this.mask = 0x00 # turn off masking on our end for now
    # We'll start by trying to find an outgoing frame
    # boundary.
    n = this._transmitRepeatedly(0x00, limit = 14)

    if n is not None:
      # We've found the packet boundary, and we know that the
      # device does not demask 0x00 into something that it
      # would silently discard.  We'll now transmit more
      # zeroes so that we can determine the high nybble of the
      # masking byte.
      n = this._measureFrameLength(0x00, limit = 14)
      if n < 13:
        maskHighNybble = n - 1
      else:
        # Complication: the high nybble of the mask can be
        # anything in 0xC..0xF.  Let's transmit 0xC0:s now;
        # these should map to frame length of 1..4.
        n = this._measureFrameLength(0xC0, limit = 5)
        if n <= 4:
          maskHighNybble = (n - 1) ^ 0xC
        else:
          raise UnableToDetermineMask
    else:
      # We have not found the packet boundary.  Assuming the
      # device is a correctly working ET-316, it must be
      # demasking our 0x00 into something that it then
      # silently discards.  Luckily, all known such values
      # have a high nybble of 0, but we should confirm by
      # transmitting 0x10:s.
      n = this._measureFrameLength(0x10, limit = 3)
      if n == 2:
        maskHighNybble = 0 # aka (n - 1) ^ 0x1
      else:
        raise UnableToDetermineMask
    
    # So, we know the mask's high nybble now.  Let's try to
    # find the low nybble by transmitting all of the [[0n]]
    # codes.  Exactly one of the responses should be [[0x05]];
    # this means that the device think we meant [[0x0E]].
    inputForFive = None
    for i in range(16):
      response = this.ask([(maskHighNybble << 4) | i],
          tolerateSilence = True)
      if response is not None:
        if len(response) > 1:
          raise UnableToDetermineMask
        if response[0] == 0x05:
          if inputForFive is None:
            inputForFive = i
          else:
            # At this point, it would seem that multiple inputs
            # in this range elicit the [[0x05]] response.  This
            # deviates from the expected protocol.
            raise UnableToDetermineMask
    if inputForFive is None:
      raise UnableToDetermineMask
    # We have found the mask.
    this.mask = (maskHighNybble << 4) | (inputForFive ^ 0x0E)
    if this.mask == 0:
      # The device has been just power-cycled, and we should
      # negotiate another mask.
      this._negotiateMask()


  def _transmitRepeatedly (this, byte, limit = None):
    i = 0
    response = this._readPacket()
    while True:
      if response is not None:
        return i # we'll discard the response
      if limit is not None and i >= limit:
        return None
      response = this.ask([byte], tolerateSilence = True)
      i += 1


  def _measureFrameLength (this, byte, limit = None):
    c1 = this._transmitRepeatedly(byte, limit = limit)
    if c1 is None:
      raise UnableToDetermineMask
    c2 = this._transmitRepeatedly(byte, limit = limit)
    if c2 is None:
      raise UnableToDetermineMask
    if c1 != c2:
      raise UnableToDetermineMask
    return c1


  # This corresponds roughly to [[CommPlus]] in [[estim.pm]]: it
  # takes the given packet (which should be either a [[bytes]],
  # a [[bytearray]], a [[list]], or a [[str]]), adds checksum if
  # it's longer than one byte, masks it, sends it to the device,
  # and reads the response (via [[Estim._readPacket]]).  If no
  # response arrives, raise [[NoResponse]] or, if
  # [[tolerateSilence]] is given as true, return [[None]].  If
  # [[expect]] is given, raise [[UnexpectedResponse]] unless the
  # response starts with this exact byte, and remove this header
  # byte before returning the packet.  The incoming packet's
  # checksum (if any) will never be included with the return
  # value.
  def ask (this, outPacket,
      tolerateSilence = False,
      expect = None):
    outPacket = bytearray(outPacket)
    if len(outPacket) > 1:
      checksum = 0
      for b in outPacket:
        checksum += b
      outPacket.append(checksum & 0xFF)
    if this.showTraffic:
      print 'TRAFFIC: -> %s (xor %02X)' % \
          (_multihex(outPacket), this.mask)
    for i in range(len(outPacket)):
      outPacket[i] ^= this.mask
    with this.lock:
      this.port.write(bytes(outPacket))
      inPacket = this._readPacket()
    if inPacket is None:
      if tolerateSilence:
        return None
      else:
        raise NoResponse, 'No response within timeout.'
    if expect is not None:
      if inPacket[0] != expect:
        raise UnexpectedResponse, \
            'Expected a %02X response but %s arrived.' % \
                (expect, _multihex(inPacket))
      else:
        # chop off the header
        inPacket = inPacket[1:]
    return inPacket


  def _readPacket (this):
    packet = this.port.read(1)
    if len(packet) == 0:
      # Timed out.
      return None
    packet = bytearray(packet)
    # The remaining byte count is the high nybble of the first
    # byte (but not all values are valid).
    rbc = packet[0] >> 4
    if rbc == 0:
      # We've got the whole packet, and there's no checksum to
      # verify.
      if this.showTraffic:
        print 'TRAFFIC: <- %s' % _multihex(packet)
      return packet
    if rbc == 1 or rbc > 12:
      raise InvalidPacketHeader, \
          'The header byte %02X is invalid.' % packet[0]
    tail = this.port.read(rbc)
    packet += bytearray(tail)
    if this.showTraffic:
      print 'TRAFFIC: <- %s' % _multihex(packet)
    if len(tail) != rbc:
      raise IncompletePacket, \
          'The packet %s is incomplete.' % _multihex(packet)
    this._verifyPacket(packet)
    # chop off the checksum and return
    return packet[:-1]


  def _verifyPacket (this, packet):
    """Check if the packet's checksum, if any.  (Single-byte
    packets do not have checksums at all.)  If the packet seems
    valid, do nothing; if bad checksum is detected, raise
    BadChecksum.  The packet should be given as a bytearray."""
    if len(packet) == 1:
      return
    expectedSum = 0
    for b in packet[:-1]:
      expectedSum += b
    expectedSum &= 0xFF
    if packet[-1] != expectedSum:
      raise BadChecksum, \
          'Packet with bad checksum from the device: %s.' % \
              _multihex(packet)


  def close (this):
    """Close the serial port to this device.  This ends the
    communication session."""
    with this.lock:
      this.port.close()


  # Since the device would activate the lowest-numbered routine
  # if we tried to activate a user routine that has not been
  # loaded to the device, this method checks the availability of
  # the user routine first if it is asked to activate a user
  # routine.
  def selectRoutine (this, number):
    """Activate a routine of the given routine number.  This
    method checks against attempting to activate a user routine
    that has not been loaded into the device.  The split routine
    can not be activated this way; use selectSplitRoutine for
    that.

    Caution: the device reinitialises the parameters upon
    routine selection instead of carrying them over from the
    previously selected routine."""
    # This is what [[ET312.java]]'s [[setCurrentMode]] does:
    if not (0x76 <= number <= 0x8E):
      raise ValueError, \
          '%i is not a valid routine number.' % number
    if number == Estim.ROUTINE_SPLIT:
      # The split routine needs special handling, and it would
      # be too messy to accommodate it here.
      raise ValueError, ('Please use selectSplitRoutine() in '
          'order to select a split routine.')
    with this.lock:
      if number >= Estim.ROUTINE_USER1:
        # When setting a user routine, we'll need to first check
        # that this routine has actually been loaded.
        # Otherwise, the device would select the lowest-numbered
        # builtin mode, [[waves]].
        if number > this.highestLoadedRoutine:
          raise IndexError, \
              'The routine user%i has not been loaded.' % \
                  (number - Estim.ROUTINE_USER1 + 1)

      this.setByte(0x407B, number - 1)
      # 0x4070 is probably a control register writing to which
      # triggers some sort of subroutine run in the device.  The
      # sleeps, explicitly present in [[ET312.java]], would give
      # the device time to complete this subroutine.
      this.setByte(0x4070, 0x04)
      time.sleep(0.018)
      # The following operation seems to be responsible for
      # incrementing the current routine's number in the
      # device's register 0x407B.  I do not know if this is its
      # only effect.
      this.setByte(0x4070, 0x10)
      time.sleep(0.018)


  # XXX: The User's Guide says (p. 16): "Not all modes are
  # available to Split as some require both channels to function
  # correctly and certain combinations may not work as
  # expected." However, I do not know what the restrictions are.
  def selectSplitRoutine (this, numberA, numberB):
    """Activate the split mode with channel A and channel B
    executing routines of the given number.  This method checks
    against attempting to activate a user routine that has not
    been loaded into the device, or attempting to activate a
    split split mode.  Note that some combinations may not be
    supported by the device (but this method does not check
    against this).  that.

    Caution: the device reinitialises the parameters upon
    routine selection instead of carrying them over from the
    previously selected routine."""
    if not (0x76 <= numberA <= 0x8E):
      raise ValueError, \
          '%i is not a valid routine number.' % number
    if not (0x76 <= numberB <= 0x8E):
      raise ValueError, \
          '%i is not a valid routine number.' % number
    if numberA == Estim.ROUTINE_SPLIT:
      raise ValueError, \
          'Multilevel split routines do not make sense.'
    if numberB == Estim.ROUTINE_SPLIT:
      raise ValueError, \
          'Multilevel split routines do not make sense.'
    with this.lock:
      if numberA >= Estim.ROUTINE_USER1 or \
          numberB >= Estim.ROUTINE_USER1:
        boundary = this.highestLoadedRoutine
        if numberA > this.highestLoadedRoutine:
          raise IndexError, \
              'The routine user%i has not been loaded.' % \
                  (numberA - Estim.ROUTINE_USER1 + 1)
        if numberB > this.highestLoadedRoutine:
          raise IndexError, \
              'The routine user%i has not been loaded.' % \
                  (numberB - Estim.ROUTINE_USER1 + 1)

      this.setByte(0x407B, Estim.ROUTINE_SPLIT - 1)
      this._splitRoutineA = numberA
      this._splitRoutineB = numberB
      this.setByte(0x4070, 0x04)
      time.sleep(0.018)
      this.setByte(0x4070, 0x10)
      time.sleep(0.018)


  def userRoutineCount (this):
    return this.highestLoadedRoutine - Estim.ROUTINE_USER1 + 1


  #### The interface to the device's comm address space

  def getByte (this, address):
    """Retrieve and return a byte from the given address of the
    device's comm address space."""
    value, = this.ask([0x3C, address >> 8, address & 0xFF],
        3, expect = 0x22)
    if this.showGets:
      print 'REG: %02X <- [%04X]' % (value, address)
    return value


  def setByte (this, address, newValue):
    """Save to the given address of the device's comm address
    space the given byte."""
    if this.showSets:
      print 'REG: %02X -> [%04X]' % (newValue, address)
    this.ask([0x4D, address >> 8, address & 0xFF, newValue],
        1, expect = Estim.RES_ACK)


  def extract (this, address, endAddress = None):
    """Extract a part of the device's comm address space from
    the given address onwards and return it as a MemoryExtract
    object.  If the end address is not given, extract a 16-byte
    part."""
    if endAddress is None:
      # By default, the extract will be 16 bytes in size.
      endAddress = address + 16
    with this.lock:
      slice = MemoryExtract(address)
      for i in range(address, endAddress):
        slice.appendByte(this.getByte(i))
      return slice

  def rom (this):
    """Extract the device's comm memory space's ROM area and
    return it as a MemoryExtract object."""
    return this.extract(0x0000, 0x0100)


  def ram (this):
    """Extract the device's comm memory space's parameter RAM
    area and return it as a MemoryExtract object."""
    return this.extract(0x4000, 0x4200)


  def nvram (this):
    """Extract the device's comm memory space's parameter NVRAM
    area and return it as a MemoryExtract object."""
    return this.extract(0x8000, 0x8200)


  #### The shortcut to the saving/restoring subsystem

  def saved (this, *names):
    return SavedTwoChannelState(this, names)


  #### Miscellania

  def show (this):
    """Show both channel's parameter registers."""
    with this.lock:
      r = this.routine
      print 'Routine: %i (%s)' % (r, decodeRoutine(r))
      print '**** Channel A ****'
      if r == Estim.ROUTINE_SPLIT:
        ra = this._splitRoutineA
        print 'Split mode channel routine: %i (%s)' % \
            (ra, decodeRoutine(ra))
      this.a.show()
      print
      print '**** Channel B ****'
      if r == Estim.ROUTINE_SPLIT:
        rb = this._splitRoutineB
        print 'Split mode channel routine: %i (%s)' % \
            (rb, decodeRoutine(rb))
      this.b.show()
      print


  def firmwareVersion (this):
    return '%i.%i.%i' % \
        (this.verMajor, this.verMinor, this.verInternalRev)


  def serial (this):
    return '%02X-%02X' % (this.serial1, this.serial2)


  #### The known global registers

  # (as in, not specific to a single channel)

  REGISTERS = {
    'verMajor': Reg(0xFD),
    'verMinor': Reg(0xFE),
    'verInternalRev': Reg(0xFF),
    'boxModel': Reg(0xFC),
    'routine': Reg(0x407B, setter = selectRoutine),
    'serial1': Reg(0x8002),
    'serial2': Reg(0x8003),
    # XXX: [[ET312.java]] seems to write these two under some
    # circumstances, apparently as a part of erasing user
    # routines
    'eLinkSig1': Reg(0x8006),
    'eLinkSig2': Reg(0x8007),
    'highestLoadedRoutine': Reg(0x8008),

    # From [[ET312.java]]
    '_splitRoutineA': Reg(0x41F5),
        # cf. [[ET312.getCurrentSplitAModeNum()]]
    '_splitRoutineB': Reg(0x41F6),
        # cf. [[ET312.getCurrentSplitBModeNum()]]

    ## Found experimentally

    # Positions of the knobs on the device's panel.  Obviously,
    # read-only.  All three are seem to run from 0x00 .. 0xFF
    # (but note that the A and B knos are displayed at the
    # device's panel as percentages, running from 00 to 99).
    'knobA': Reg(0x4064, setter = None),
    'knobB': Reg(0x4065, setter = None),
    # Copies of the knobMA seem to be stored in all of the bytes
    # at 0x41C0..0x41CF.
    'knobMA': Reg(0x4061, setter = None),

    # These preferences, in the region of [[0x41F4..0x41F9]]
    # (and possibly a few further ones around them) seem to have
    # a corresponding persistent preference region in the NVRAM,
    # at [[0x8009..0x900E]].
    'ramPowerLevel': Reg(0x41F4, threshold = 1, top = 3),

    'ramFavSplitRoutineA': Reg(0x41F5),
    'ramFavSplitRoutineB': Reg(0x41F6),
    'ramFavRoutine': Reg(0x41F7),
        # Note that [[0x4078]] tends to vary together with this.
        # This is probably an artefact; [[0x4078]] looks likely
        # to hold the routine currently chosen, but not
        # necessarily activated, in the device's panel menu
        # system.

    # [[rampLevel]] runs, nominally, from 50 (0xCD) to 100
    # (0xFF) by increments of 10.  Because our registers'
    # user-friendly values start from zero, the permitted range
    # is 0..50.
    'ramRampLevel': Reg(0x41F8, threshold = 0xCD),
    # The nominal minimum value for the rampTime is 1 (and
    # indeed, zero would almost certainly make any sense at
    # all).  I'm not restricting the zero here, though.
    'ramRampTime': Reg(0x41F9, top = 120),

    'powerLevel': Reg(0x8009, threshold = 1, top = 3),
    'favSplitRoutineA': Reg(0x800A),
    'favSplitRoutineB': Reg(0x800B),
    'favRoutine': Reg(0x800C),
    'rampLevel': Reg(0x800D, threshold = 0xCD),
    'rampTime': Reg(0x800E, top = 120),
  }


#### The channel objects

class Channel (AbstractRegisterBank):

  def __init__ (this, estim, number, ignoreDisablers = False):
    if number not in (0, 1):
      raise ValueError, (
          'The value %s does not specify a known channel: '
          'either 0 or 1.' % `number`)
    object.__init__(this)
    this.estim = estim
    this.lock = estim.lock
    this.number = number
    this.ignoreDisablers = ignoreDisablers
    # The first register-specific parameter is not addressed at
    # [[0x4000]], nor [[0x4100]].  However, we use these round
    # addresses for convenience.
    this.origin = [0x4000, 0x4100][this.number]


  # Let [[__setattr__]] know of our defined attributes:
  KNOWN_ATTR = 'estim', 'number', 'origin', 'lock', \
               'ignoreDisablers'


  #### Byte addressing relative to this channel's origin

  def getByte (this, offset):
    return this.estim.getByte(this.origin + offset)


  def setByte (this, offset, newValue):
    this.estim.setByte(this.origin + offset, newValue)


  #### The shortcut to the saving/restoring subsystem

  def saved (this, *names):
    return SavedChannelState(this, names)

  #### Option register accessors

  # There are five option registers per channel, and they're a
  # bit tricky to both encode and decode.

  # If the timer value (two bottom bits) of the given raw option
  # register value is nonzero, reset it at 1; otherwise, keep it
  # as zero.
  def _initTimerValue (this, rawValue):
    # We'll check if bit 1 is set: if not, the field holds
    # either 0 or 1 and nothing needs to be done; otherwise, the
    # field holds either 2 or 3 and we'll need to reset it to 1.
    if (rawValue & 0x02) != 0x00:
      rawValue = (rawValue & ~0x03) | 0x01
    return rawValue


  ## Gate option accessors

  def getTimeOptions (this):
    rawValue = this._timeOpt
    onOpt = rawValue & 0x0C
    offOpt = (rawValue & 0x60) >> 4
    return onOpt, offOpt


  def setTimeOptions (this, newValue):
    onOpt, offOpt = newValue

    # The UI for adjusting time options, 'gate', has four
    # checkboxes.  For the 'on' nybble, there are
    # [[gateValMACheckBox]] (0x08, "MA") and
    # [[gateValAdvPCheckBox]] (0x04, "Effect").  (When setting
    # either, both two bottom bits should be cleared.)  For the
    # 'off' nybble, there are [[gateRateMACheckBox]] (0x40,
    # "MA") and [[gateRateAdvPCheckBox]] (0x20, "Tempo").
    #
    # XXX: the Programmer's Manual suggests setting the bottom
    # two bits to [[01]] whenever storing to this register, but
    # this is inconsistent with the Java app.
    with this.estim.lock:
      rawValue = this._timeOpt

      if onOpt not in (0, 4, 8):
        raise ValueError, ('Invalid on-nybble input '
            'for timeOpt: %s') % repr(onOpt)
      if (rawValue & 0x0C) != onOpt:
        # Bottom nybble is changing
        rawValue = (rawValue & ~0x0C) | onOpt
        rawValue = this._initTimerValue(rawValue)
        if onOpt != 0:
          # If a checkbox in this box got checked, we'll clear
          # the timer value field.
          rawValue &= ~0x03

      # XXX: In case of error, [[_replaceRateOptionField]] says
      # that it expects /rate/ nybble rather than /off/ nybble.
      # It may be a problem, but it may also be that the off
      # nybble is a special case of the more general rate
      # nybble.  For now, I'm considering this as probably not
      # an issue or perhaps a purely cosmetic issue.
      rawValue = _replaceRateOptionField(rawValue, offOpt)

      this._timeOpt = rawValue


  ## Level option accessors

  def getLevelOptions (this):
    rawValue = this._levelOpt
    minOpt = rawValue & 0x0C
    rateOpt = (rawValue & 0x60) >> 4
    return minOpt, rateOpt


  def setLevelOptions (this, newValue):
    minOpt, rateOpt = newValue

    # The UI for adjusting level options, 'level', has three
    # checkboxes.  For the 'min' nybble, there's
    # [[levelMinAdvPCheckBox]] (0x04, "Depth").  For the 'rate'
    # nybble, there are levelRateMACheckBox (0x40, "MA") and
    # levelRateAdvPCheckBox (0x20, "Tempo").
    with this.estim.lock:
      rawValue = this._levelOpt

      if minOpt not in (0, 4):
        raise ValueError, ('Invalid min-nybble input '
            'for levelOpt: %s') % repr(minOpt)
      if (rawValue & 0x0C) != minOpt:
        # Bottom nybble is changing
        rawValue = (rawValue & ~0x0C) | minOpt
        rawValue = this._initTimerValue(rawValue)

      rawValue = _replaceRateOptionField(rawValue, rateOpt)

      this._levelOpt = rawValue


  ## Frequency option accessors

  # The frequency options register is an interesting case
  # because its UI uses all six of the checkboxes.

  def getFrequencyOptions (this):
    rawValue = this._frequencyOpt
    # extract the two main bits of the value option nybble
    valOpt = rawValue & 0x0C
    # add one if the timer value field is nonzero, except if the
    # main two bits value option field are zero
    if valOpt != 0 and (rawValue & 0x03) != 0:
      valOpt += 1
    rateOpt = (rawValue & 0x60) >> 4
    return valOpt, rateOpt


  def setFrequencyOptions (this, newValue):
    valOpt, rateOpt = newValue

    with this.estim.lock:
      rawValue = this._frequencyOpt

      if valOpt not in (0, 4, 5, 8, 9):
        raise ValueError, ('Invalid val-nybble input '
            'for frequencyOpt: %s') % repr(valOpt)
      if (valOpt == 0):
        # If 'none', we'll retain the bottom two bits, the
        # 'timer value'.
        rawValue = (rawValue & ~0x0C)
      else:
        # Otherwise, the bottom two bits get initialised from
        # the passed nybble.
        rawValue = (rawValue & ~0x0F) | valOpt

      rawValue = _replaceRateOptionField(rawValue, rateOpt)

      this._frequencyOpt = rawValue


  ## Width option accessors

  def getWidthOptions (this):
    rawValue = this._widthOpt
    # extract the two main bits of the value option nybble
    valOpt = rawValue & 0x0C
    # add one if the timer value field is nonzero, except if the
    # main two bits value option field are zero
    if valOpt != 0 and (rawValue & 0x03) != 0:
      valOpt += 1
    rateOpt = (rawValue & 0x60) >> 4
    return valOpt, rateOpt


  def setWidthOptions (this, newValue):
    valOpt, rateOpt = newValue

    with this.estim.lock:
      rawValue = this._widthOpt

      if valOpt not in (0, 4, 5):
        raise ValueError, ('Invalid val-nybble input '
            'for widthOpt: %s') % repr(valOpt)
      if (valOpt == 0):
        # If 'none', we'll retain the bottom two bits, the
        # 'timer value'.
        rawValue = (rawValue & ~0x0C)
      else:
        # Otherwise, the bottom two bits get initialised from
        # the passed nybble.
        rawValue = (rawValue & ~0x0F) | valOpt

      rawValue = _replaceRateOptionField(rawValue, rateOpt)

      this._widthOpt = rawValue


  #### Disablers

  # In the Java UI, the sliders are subject to slightly
  # nontrivial enable/disable rules.  Most, and perhaps all, of
  # them seem to be inspired by what makes sense to change in
  # any particular routine and what doesn't, and therefore, it's
  # desirable to also implement the disabledness rules in the
  # Python library.  We'll do this by implementing their
  # corresponding registers in two levels: [[_foo]] will be the
  # raw register without disabling logic, and [[foo]] will be
  # the corresponding register that, when assigned to when it
  # should not be, will raise [[RegisterCurrentlyDisabled]].
  # NOTE though a downside to implementing disabledness rules in
  # this way: every assignment takes a device memory read before
  # a write operation, so a 'guarded' assignment takes roughly
  # twice the time that a 'raw' assignment does.  Considering
  # the low communication speed, this may be a significant issue
  # when setting many parameters at once.  However, let's not
  # prematurely try to optimise this complex case before the
  # issues actually show up.

  ## The value sliders

  # The 'gate select' variant of value slider disabler
  def timeOnDisabler (this):
    # (cf. [[SelectUpdater.useGateSelectLogic]])
    b = this._timeOpt
    return (b & 0x03) == 0 or (b & 0x0C) != 0


  # The disabler for other value sliders.  (The guard is
  # specified by its register name.)
  @classmethod
  def valSliderDisabler (cls, guard):
    def disabler (this):
      b = getattr(this, guard)
      return (b & 0x03) == 0 and (b & 0x0C) != 0
    return disabler


  ## The minimum value sliders

  @classmethod
  def minSliderDisabler (cls, guard):
    def disabler (this):
      b = getattr(this, guard)
      return (b & 0x03) == 0x00 or (b & 0x0C) != 0x00
    return disabler


  ## The maximum value sliders

  @classmethod
  def maxSliderDisabler (cls, guard):
    def disabler (this):
      b = getattr(this, guard)
      return (b & 0x03) == 0x00
    return disabler


  ## The rate sliders

  @classmethod
  def rateSliderDisabler (cls, guard):
    def disabler (this):
      b = getattr(this, guard)
      # The RHS of this 'or' means "Either of the rate option
      # flags has been activated".
      return (b & 0x03) == 0 or (b & 0x60) != 0
    return disabler


  #### User-friendly option selectors

  # NOTE that these don't set the _whole_ nybbles, just the
  # major fields that coincide with one of the two nybbles.

  def _setLowNybble (this, name, newValue):
    low, high = getattr(this, name)
    low = newValue
    setattr(this, name, (low, high))


  def _setHighNybble (this, name, newValue):
    low, high = getattr(this, name)
    high = newValue
    setattr(this, name, (low, high))


  ## The 'time' (aka 'gate') option register

  def clearTimeOnFlags (this):
    this._setLowNybble('timeOpt', 0)
  def selectTimeOnEffect (this):
    this._setLowNybble('timeOpt', 4)
  def selectTimeOnMA (this):
    this._setLowNybble('timeOpt', 8)


  def clearTimeOffFlags (this):
    this._setHighNybble('timeOpt', 0)
  def selectTimeOffTempo (this):
    this._setHighNybble('timeOpt', 2)
  def selectTimeOffMA (this):
    this._setHighNybble('timeOpt', 4)


  ## The 'level' option register

  def clearLevelMinFlags (this):
    this._setLowNybble('levelOpt', 0)
  def selectLevelMinDepth (this):
    this._setLowNybble('levelOpt', 4)


  def clearLevelRateFlags (this):
    this._setHighNybble('levelOpt', 0)
  def selectLevelRateTempo (this):
    this._setHighNybble('levelOpt', 2)
  def selectLevelRateMA (this):
    this._setHighNybble('levelOpt', 4)


  ## The 'frequency' option register

  def clearFrequencyValueFlags (this):
    this._setLowNybble('frequencyOpt', 0)
  def selectFrequencyValueFreq (this):
    this._setLowNybble('frequencyOpt', 4)
  def selectFrequencyMaxFreq (this):
    this._setLowNybble('frequencyOpt', 5)
  def selectFrequencyValueMA (this):
    this._setLowNybble('frequencyOpt', 8)
  def selectFrequencyMaxMA (this):
    this._setLowNybble('frequencyOpt', 9)


  def clearFrequencyRateFlags (this):
    this._setHighNybble('frequencyOpt', 0)
  def selectFrequencyRateEffect (this):
    this._setHighNybble('frequencyOpt', 2)
  def selectFrequencyRateMA (this):
    this._setHighNybble('frequencyOpt', 4)


  ## The 'width' option register

  def clearWidthValueFlags (this):
    this._setLowNybble('widthOpt', 0)
  def selectWidthValueWidth (this):
    this._setLowNybble('widthOpt', 4)
  def selectWidthMinWidth (this):
    this._setLowNybble('widthOpt', 5)


  def clearWidthRateFlags (this):
    this._setHighNybble('widthOpt', 0)
  def selectWidthRatePace (this):
    this._setHighNybble('widthOpt', 2)
  def selectWidthRateMA (this):
    this._setHighNybble('widthOpt', 4)


  #### Overview of channel state

  def show (this):
    """Show this channel's parameter registers."""
    regs = []
    maxNameLen = 0
    for k, v in Channel.REGISTERS.items():
      # skip the 'raw' registers
      if k[0] != '_':
        regs.append((v.offset, k, v))
      maxNameLen = max(maxNameLen, len(k))
    regs.sort()
    template = '0x%%04X %%%is =' % maxNameLen
    with this.lock:
      for offset, name, reg in regs:
        value = getattr(this, name)
        # NOTE that the value can be either an integer or a
        # tuple of two integers.
        print template % (this.origin + offset, name),
        if type(value) is not tuple:
          # The space before but not after the slash hints that
          # it acts as a preposition.  After all, the slash
          # should be pronounced 'out of' here.
          print '%3i /%i' % (value, reg.upperBound()),
        else:
          print repr(value),
        if reg.disabler:
          if reg.disabler(this):
            print '(disabled)',
        print


#### Register list

Channel.REGISTERS = {
  # The main output signal control registers for each channel
  # are divided into four blocks: gate, level, frequency, and
  # width.  Each block contains one 'options' register, which
  # has complicated internal structure; the other registers
  # just hold an integer in a particular range.

  ## Gate registers

  # As per [[ET312InteractiveChannelPanel.java]], the
  # [[timeOn]] and [[timeOff]] registers have zero
  # threshold.  This is in agreement with the Programmer's
  # Manual and [[estim.pl]].
  '_timeOn': Reg(0x98),
  'timeOn': Reg(0x98, disabler = Channel.timeOnDisabler),
  '_timeOff': Reg(0x99),
  'timeOff': Reg(0x99,
      disabler = Channel.rateSliderDisabler('_timeOpt')),

  '_timeOpt': Reg(0x9A),
  'timeOpt': Reg(0x9A, # XXX: "See note 1"
      getter = Channel.getTimeOptions,
      setter = Channel.setTimeOptions),
      # (on, off)
      # on: 0 = default, 4 = effect, 8 = MA
      # off: 0 = default, 2 = tempo, 4 = MA

  ## Level registers

  # XXX: According to the Programmer's Manual and
  # [[estim.pl]], the level should be clipped to (0x80 ..
  # 0xFF).  However, the Java code's level, levelMin, and
  # levelMax sliders run from 127 to 255.  I'm going with what
  # Java does here.
  '_level': Reg(0xA5, 127),
  'level': Reg(0xA5, 127,
      disabler = Channel.valSliderDisabler('_levelOpt')),
  'levelMin': Reg(0xA6, 127,
      disabler = Channel.minSliderDisabler('_levelOpt')),
  'levelMax': Reg(0xA7, 127,
      disabler = Channel.maxSliderDisabler('_levelOpt')),

  # XXX: The Programmer's Manual specifies a range of
  # 0x08..0xFF for [[b.levelRate]] but 0x00..0xFF for
  # [[a.levelRate]].  [[estim.pl]] does not impose a
  # threshold, using 0x00..0xFF as the range for both.
  # [[ET312InteractiveChannelPanel.java]] uses 1..255 as the
  # range.  I'm going with what Java does here.
  'levelRate': RevReg(0xA8, 1,
      disabler = Channel.rateSliderDisabler('_levelOpt')),

  '_levelOpt': Reg(0xAC),
  'levelOpt': Reg(0xAC,
      getter = Channel.getLevelOptions,
      setter = Channel.setLevelOptions),
      # (min, rate)
      # min: 0 = normal, 4 = depth
      # rate: 0 = normal, 2 = tempo, 4 = MA.
      #
      # XXX: the Programmer's manual declares a nybble order
      # that would translate to (rate, min) here, but based on
      # the Java code, this seems to be an error.
      # I'm going with the order consistent with [[estim.pl]]
      # and the Java app.
      #
      # XXX: The Programmer's Manual suggests a reverse
      # meaning for the [[min]] values (i.e., 4 = normal, 0 =
      # depth), but based on the Java code, this seems to be
      # an error.
      #
      # XXX: [[estim.pm]] suggests adding one to the 'min'
      # value (i.e., 1 = normal, 5 = depth.)  This may be
      # accurate, in that the two bottom bits, the 'timer
      # value', seem to generally have nonzero values in
      # context of min checkboxes, but I implement this via
      # [[_initTimerValue]], and the caller does not have to
      # explicitly supply a nonzero timer value field.
      #
      # XXX: [[estim.pm]] and notes 4a and 4b of the
      # Programmer's Manual suggest adding 1 to the 'rate'
      # value for channel A but not for channel B.  Based on
      # the Java code, this seems to be an error -- this 1
      # would actually fall outside the 'rate' field proper,
      # and it seems to be a device-managed flag.  I'm not
      # expecting this flag from the caller, and instead
      # retain whatever was in the register before.

  ## Frequency registers

  # The threshold for the three frequency registers is 8, as
  # per [[ET312InteractiveChannelPanel.java]].
  # XXX: There seems to be a typo in the Programmer's Manual
  # -- it specifies an out-of-pattern range of 0x00..0xFF for
  # [[b.frequencyMax]].  The general pattern for these three,
  # on both channels, is [[0x08..0xFF]].
  'frequency': RevReg(0xAE, 0x08,
      disabler = Channel.valSliderDisabler('_frequencyOpt')),
  'frequencyMax': RevReg(0xAF, 0x08,
      disabler = Channel.maxSliderDisabler('_frequencyOpt')),
  'frequencyMin': RevReg(0xB0, 0x08,
      disabler = Channel.minSliderDisabler('_frequencyOpt')),

  # As per [[ET312InteractiveChannelPanel.java]], the
  # threshold for [[frequencyRate]] is 1.
  'frequencyRate': RevReg(0xB1, 0x01,
      disabler = Channel.rateSliderDisabler('_frequencyOpt')),

  '_frequencyOpt': Reg(0xB5),
  'frequencyOpt': Reg(0xB5,
      getter = Channel.getFrequencyOptions,
      setter = Channel.setFrequencyOptions),
      # (value, rate)
      # - value: 0 = none, 4 = val/freq, 5 = max/freq,
      #   8 = val/MA, 9 = max/MA
      # - rate: 0 = none, 2 = effect, 4 = MA
      # (sources: note 2 of the Programmer's Manual,
      # [[estim.pm]], and the Java app)


  ## Width registers

  # The threshold for the three width registers is 70, as per
  # [[ET312InteractiveChannelPanel.java]]
  # XXX: the Programmer's Manual specifies 0x40 (= 64), and
  # [[estim.pl]] does likewise.
  # XXX: The User Guide states (p. 19) that the range of
  # width is 70-250 microseconds.
  # XXX: I have seen values as low as 68 raw (-2 adjusted) for
  # width and 50 raw (-20 adjusted) for widthMin, apparently set
  # by the device itself.
  'width': Reg(0xB7, 70,
      disabler = Channel.valSliderDisabler('_widthOpt')),
  'widthMin': Reg(0xB8, 70,
      disabler = Channel.minSliderDisabler('_widthOpt')),
  'widthMax': Reg(0xB9, 70,
      disabler = Channel.maxSliderDisabler('_widthOpt')),

  # The threshold for [[widthRate]] is 1, as per
  # [[ET312InteractiveChannelPanel.java]].
  # XXX: The Programmer's Manual specifies 0x00, and
  # [[estim.pl]] does likewise.
  'widthRate': RevReg(0xBA, 1,
      disabler = Channel.rateSliderDisabler('_widthOpt')),

  '_widthOpt': Reg(0xBE),
  'widthOpt': Reg(0xBE,
      getter = Channel.getWidthOptions,
      setter = Channel.setWidthOptions),
      # (value, rate)
      # - value: 0 = none, 4 = val/width, 5 = min/width
      # - rate: 0 = none, 2 = pace, 4 = MA
      #
      # XXX: the note 3 of the Programmer's Manual and
      # [[estim.pm]] suggest using 1 for the value field.
      # Instead, for consistency, we're using 0 for the 'none'
      # value and when it's used, keep the previous timer
      # value field -- the two bottom bits of the register --
      # intact.

  ## The (unusual) routine number register

  # The routine number does not semantically seem to be a
  # channel register, but it lies in the address space of
  # Channel A.
  #
  # XXX: I'd speculate that in the mode of [[ROUTINE_SPLIT]],
  # the Channel B's routine number might be saved at the same
  # offset in the address space of Channel B, but this has not
  # been tested.
  #
  # NOTE that merely storing a new value into this register
  # does not actually activate a new routine; in order two
  # achieve this, two further control pokes must be performed.
  # Use [[Estim.selectRoutine]] (or equivalently, assign to
  # [[Estim.routine]]) instead of messing with [[_routine]]
  # directly.
  '_routine': Reg(0x7B),

  ## Observations on the option registers
  # - when the UI checkboxes for these options are changed,
  # bits 4 and 7 (mask 0x90) are always retained

  # For each group, up to six checkboxes are defined (although
  # some are [[null]] for any particular group).  These fall
  # into two groups:
  # - [[valMACheckBox]] (0x08), [[valAdvPCheckBox]] (0x04),
  #   [[minMACheckBox]] (0x08), [[minAdvPCheckBox]] (0x04);
  # - [[rateMACheckBox]] (0x40), [[rateAdvPCheckBox]] (0x20).
  # In each group, selecting one of these checkboxes deselects
  # all the other checkboxes in the same group.  It's
  # permissible for all the checkboxes in a group to be
  # deselected.
  #
  # In the first group, there are four checkboxes but only two
  # distinct values.  The differentiating factor is in the two
  # bottom bits.
  #
  # It seems that whenever an option bit is stored at the
  # device, the previous value should be fetched first.  The
  # bits of 0x90 should be retained.  The two bottom bits
  # (which a local variable in the Java code calls "timer
  # value") are a bit tricky: If valAdvPCheckBox or
  # valMACheckBox is selected in the new configuration, the
  # two bottom bits should be set to [[00]].  Otherwise, the
  # two bottom bits should be set to [[01]] EXCEPT if they're
  # previously [[00]].  (The Java code caches the register
  # value from the last UI change instead of loading it before
  # store, though.)

  # There are four groups:
  # - 'gate' ("Timing" in the UI) has valMACheckBox ("MA"),
  #   valAdvPCheckBox ("Effect"); rateMACheckBox ("MA"),
  #   rateAdvPCheckBox ("Tempo");
  # - 'level' has minAdvPCheckBox ("Depth"); rateMACheckBox
  #   ("MA"), rateAdvPCheckBox ("Tempo");
  # - 'freq' has valMACheckBox ("MA"), valAdvPCheckBox
  #   ("Freq"), minMACheckBox ("MA"), minAdvPCheckBox
  #   ("Freq"); rateMACheckBox ("MA"), rateAdvPCheckBox
  #   ("Effect");
  # - 'width' has valAdvPCheckBox ("Width"), minAdvPCheckBox
  #   ("Width"); rateMACheckBox ("MA"), rateAdvPCheckBox
  #   ("Pace").

  # Also note that the four sliders of a group can be disabled
  # based on bits from the options register.  Some of these
  # bits are user-controlled; others are device-controlled.
  # - All four sliders of a group are disabled if the two
  #   bottom bits of the options register are [[00]].  These
  #   seem to be "timing value"; I do not know the exact
  #   significance.
  # - In the 'gate' group, the value slider is also disabled
  #   if [[(options & 0xC) != 0x00]].
  # - The min slider is also disabled if [[(options & 0xC) !=
  #   0x00]].
  # - The rate slider is also disabled if [[(options & 0x60)
  #   != 0x00]].
}


#### The saved state objects

class SavedChannelState (object):
  def __init__ (this, channel, names):
    object.__init__(this)
    this.channel = channel
    # Note that acquiring a lock at our end does not prevent a
    # routine that may be active on the device from affecting
    # the parameters in the middle of saving.
    with channel.estim.lock:
      this.values = [(n, getattr(channel, n)) for n in names]

  def restore (this):
    # Note that acquiring a lock at our end does not prevent a
    # routine that may be active on the device from affecting
    # the parameters in the middle of restoring.
    with this.channel.estim.lock:
      for name, value in this.values:
        setattr(this.channel, name, value)

  ## The [[with]] interface to [[SavedChannelState]]

  def __enter__ (this):
    pass

  def __exit__ (this, type, value, traceback):
    this.restore()
    # Re-raise any exception that might have been in flight:
    return None


class SavedTwoChannelState (object):
  def __init__ (this, estim, names):
    object.__init__(this)
    this.estim = estim
    with estim.lock:
      this.A = SavedChannelState(estim.A, names)
      this.B = SavedChannelState(estim.B, names)

  def restore (this):
    with this.estim.lock:
      this.a.restore()
      this.b.restore()

  ## The [[with]] interface to [[SavedTwoChannelState]]

  def __enter__ (this):
    pass

  def __exit__ (this, type, value, traceback):
    this.restore()
    # Re-raise any exception that might have been in flight:
    return None


#### Utility methods

def _swapByteNybbles (b):
  return ((b >> 4) | (b << 4)) & 0xFF


def _multihex (blob):
  """Prepare a hex dump of binary data, given in any common form
  including [[str]], [[list]], [[bytes]], or [[bytearray]]."""
  return ' '.join(['%02X' % b for b in bytearray(blob)])


# A previous version of [[estimlib.py]] defined
# [[Estim.selectUserRoutine()]] instead.  However, this does not
# work well together with support for split mode selection,
# which we'll hopefully soon have.  Thus, [[userRoutine()]] is
# now a utility method.
def userRoutine (this, number):
  """Translate a number in the range of 1..7 into an ET-312B
  routine code for the user-defined routine of this number."""
  if not (1 <= number <= 7):
    raise ValueError, \
        '%i is not a valid user routine number.' % number
  return number + Estim.ROUTINE_USER1 - 1


def _replaceRateOptionField (rawValue, rateOpt):
  if rateOpt not in (0, 2, 4):
    raise ValueError, \
        'Invalid rate-nybble input' % repr(rateOpt)
  return (rawValue & ~0x60) | (rateOpt << 4)


def decodeRoutine (code):
  """Given a routine code, return its name as a human-readable
  string.  Returns a single question sign if the code does not
  correspond to any known routine."""
  try:
    return ROUTINE_NAMES[code]
  except KeyError:
    return '?'


ROUTINE_NAMES = {
  0x00: 'none',
  0x76: 'waves',
  0x77: 'stroke',
  0x78: 'climb',
  0x79: 'combo',
  0x7A: 'intense',
  0x7B: 'rhythm',
  0x7C: 'audio1',
  0x7D: 'audio2',
  0x7E: 'audio3',
  0x7F: 'split',
  0x80: 'random1',
  0x81: 'random2',
  0x80: 'toggle',
  0x83: 'orgasm',
  0x84: 'torment',
  0x85: 'phase1',
  0x86: 'phase2',
  0x87: 'phase3',
  0x88: 'user1',
  0x89: 'user2',
  0x8A: 'user3',
  0x8B: 'user4',
  0x8C: 'user5',
  0x8D: 'user6',
  0x8E: 'user7',
}


#### The exception hierarchy

class EstimError (RuntimeError):
  pass

class CommFailure (EstimError):
  pass

class NoResponse (CommFailure):
  pass

class IncompletePacket (CommFailure):
  pass

class UnexpectedResponse (CommFailure):
  pass

class UnableToDetermineMask (CommFailure):
  pass

class InvalidPacketHeader (CommFailure):
  pass

class BadChecksum (CommFailure):
  pass

class NakLoop (CommFailure):
  pass

class EstimConstraintViolation (EstimError):
  pass

# XXX: Attempts to write to the read-only memory range currently
# produce [[UnexpectedResponse]], for the device will NAK
# instead of ACK them.
class RegisterNotWritable (EstimConstraintViolation):
  pass

class RegisterCurrentlyDisabled (EstimConstraintViolation):
  pass


#### Helpers for analysing the device's comm memory layout

class MemoryExtract (object):
  def __init__ (this, origin, data = []):
    object.__init__(this)
    this.origin = origin
    this.data = bytearray(data)


  def __getitem__ (this, address):
    offset = address - this.origin
    if offset < 0:
      raise IndexError
    return this.data[offset]
    

  def appendByte (this, value):
    """Append the given byte to this memory extract."""
    this.data.append(value)


  def dump (this):
    """Print a standard hex dump of this memory extract to the
    standard output."""
    endAddr = this.origin + len(this.data)
    for rowAddr in range(this.origin, endAddr, 0x10):
      hexOutput = []
      charOutput = ''
      for i in range(rowAddr, rowAddr + 16):
        if i < endAddr:
          b = this[i]
          hexOutput.append('%02X' % b)
          if 0x20 <= b <= 0x7E:
            charOutput += chr(b)
          else:
            charOutput += '.'
        else:
          # we've reached the endAddr
          hexOutput.append('  ') # padding
          # charOutput does not need to be padded
      print '%04X:' % rowAddr, \
          ' '.join(hexOutput), charOutput


  def __eq__ (this, that):
    return this.data == that.data


  def differ (this, *those):
    """Given this memory extract and one or more additional
    memory extracts of the same size, list all the locations --
    and their corresponding values -- where differences between
    these extracts occur."""
    # Note that we won't compare origins.
    for that in those:
      if len(this.data) != len(that.data):
        raise RuntimeError, 'size mismatch'
    all = [this] + list(those)
    for i in range(len(this.data)):
      allEqual = True
      for that in those:
        if this.data[i] != that.data[i]:
          allEqual = False
          break
      if not allEqual:
        print '%04X:' % (this.origin + i),
        for that in all:
          print '%02X' % that.data[i],
        print


  def growing (this, *those):
    """Given this memory extract and one or more additional
    memory extracts of the same size, list all the locations --
    and their corresponding values -- where each subsequent
    memory extract has a strictly higher value than all its
    predecessors."""
    for that in those:
      if len(this.data) != len(that.data):
        raise RuntimeError, 'size mismatch'
    all = [this] + list(those)
    for i in range(len(this.data)):
      allUpwards = True
      for j in range(1, len(all)):
        if all[j - 1].data[i] >= all[j].data[i]:
          allUpwards = False
          break
      if allUpwards:
        print '%04X:' % (this.origin + i),
        for that in all:
          print '%02X' % that.data[i],
        print


  def find (this, *values):
    """Show all the locations (and values) in this memory
    extract where any of the given exact byte values appears."""
    for i in range(len(this.data)):
      if this.data[i] in values:
        print '%04X: %02X' % (this.origin + i, this.data[i])


  def write (this, filename):
    """Write the raw binary content of this memory extract into
    the given file.  (Note that the extract's origin address
    will be lost.)"""
    with open(filename, 'wb') as f:
      f.write(this.data)


  @classmethod
  def read (cls, filename, origin = 0x0000):
    """Read the given file's raw binary content and package it
    up as a new MemoryExtract instance, optionally using the
    given origin address."""
    with open(filename, 'rb') as f:
      return cls(origin, f.read())


#### The interactive user interface

if __name__ == '__main__':
  from code import InteractiveConsole
  import getopt
  import sys

  try:
    options, args = getopt.getopt(sys.argv[1:],
        'p:tm:idh',
        ['port=', 'traffic', 'registers', 'mask-file=',
        'interact', 'dump', 'help'])
  except getopt.GetoptError as e:
    print str(e)
    sys.exit(1)

  mode = 'interact'
  showTraffic = False
  showRegisters = False
  port = None
  saveMask = None

  for o, a in options:
    if o in ('-d', '--dump'):
      mode = 'dump'
    elif o in ('-p', '--port'):
      port = a
    elif o in ('-t', '--traffic'):
      showTraffic = True
    elif o in ('-r', '--registers'):
      showTraffic = True
    elif o in ('-m', '--mask-file'):
      saveMask = a
    elif o in ('-i', '--interact'):
      mode = 'interact'
    elif o in ('-h', '--help'):
      print USAGE
      exit(0)

    else:
      assert False

  e = estim = Estim(port,
      saveMask = saveMask,
      showTraffic = showTraffic,
      showGets = showRegisters,
      showSets = showRegisters)

  if mode == 'dump':
    if len(args) == 0:
      # By default, we'll dump two of the three potentially
      # interesting memory regions.  The third one,
      # [[0000:0100]], seems to be immutable and thus does not
      # need to be routinely dumped.
      args = '4000:4200', '8000:8200'
    for region in args:
      # The region can be given as a single hex number or two
      # colon-separated hex numbers.  In either case, the first
      # number is the start address.  If the second number is
      # given, it's the address just after the last address to
      # dump.  If it's not given, 256 bytes are dumped.
      regionBounds = region.split(':', 1)
      startAddr = int(regionBounds[0], 16)
      if len(regionBounds) >= 2:
        endAddr = int(regionBounds[1], 16)
      else:
        endAddr = startAddr + 0x100
      estim.extract(startAddr, endAddr).dump()
      print

  elif mode == 'interact':
    print 'Firmware version: %s' % estim.firmwareVersion()
    print 'Box model: %i' % estim.boxModel
    # The serial number space is strangely small but assuming
    # different devices have different serial numbers, the
    # serial numbers may nevertheless be useful in a
    # multi-device situation to distinguish the devices even if
    # they move around a set of hardware ports.
    print 'Serial number: %s' % estim.serial()
    urc = estim.userRoutineCount()
    if urc == 1:
      print 'The device has 1 user routine loaded.'
    else:
      print 'The device has %i user routines loaded.' % \
          urc
    print
    InteractiveConsole(locals = globals()).interact(
        'Entering interactive Python environment.\n'
        'The Estim instance can be accessed via the global\n'
        "variable 'estim', or 'e' for short.\n")

  else:
    assert False

  e.close()
  exit()

