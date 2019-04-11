import gpiozero
from gpiozero import Button
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import pigpio
from dorna import Dorna
import time
robot = Dorna()
pi = pigpio.pi()
factory = PiGPIOFactory(host='169.254.232.97')

def flipStart(factory):

    # Purpose:  Raise pneumatics at start of sequence
    #           Flip phone over on crack platform

    # Assume:   Arm starts in home orientation
    #           LA starts at home
    #           Vacuum pump off
    #           Phone placed on crack platform (either side face up)

    # End:      Arm in home orientation
    #           LA at home
    #           Vacuum pump off
    #           Phone flipped over
    #           Pneumatics raised

    # Inputs:   factory (IP address)

    # Outputs:  True if successful (done command)
    #           False if unsuccessful (kill command)

    # Arm movements list
    moveHome = ['xh', 'yh', 'zh', 'ah', 'bh']
    moveFlipC = ['xc', 'yc', 'zc', 'ac', 'bc']
    moveFlip0 = ['x0', 'y0', 'z0', 'a0', 'b0']
    moveFlip1 = ['x1', 'y1', 'z1', 'a1', 'b1']
    moveFlip2 = ['x2', 'y2', 'z2', 'a2', 'b2']

    # Raise pneumatics
    raiseLower(True, factory)

    # Center LA
##    if LAMove(moveFlipC, factory) == False:
##        return False
    
    # Home Arm
    robot.home("j1")
    robot.home("j2")
    robot.home("j3")
    print(robot.homed())

    # Move arm to its home orientation
##    dornaMove(moveHome)

    # Send LA to its home position
    if LAHome(factory) == False:
        return False
    
    # Turn on vacuum
    vpump(True, factory)

    # Move arm to top of crack platform
##    dornaMove(moveFlip0)

    # Slowly lower effector until arm transducer reads vacuum
    if slowApproachArm(factory) == False:
        return False
    
    # Slide phone off of platform
##    if LAMove(moveFlip0, factory) == False:
##        return False

    # Flip phone over and 
##    dornaMove(moveFlip1)

    # Check phone hasn't been accidentally released
    if ArmTrans(factory) == False:
        return False

    # Slide phone onto platform
    if LAHome(factory) == False:
        return False

    # Release vacuum
    vpump(False, factory)

    # Check phone has been released
    if ArmTrans(factory) == True:
        return False

    # Move arm out of the way
##    dornaMove(moveFlip2)

    # Move arm to its home orientation
##    dornaMove(moveHome)

    return True

def flip(factory):

    # Purpose:  Flip phone over on crack platform

    # Assume:   Arm starts in home orientation
    #           LA starts at home
    #           Vacuum pump off
    #           Phone placed on crack platform (either side face up)

    # End:      Arm in home orientation
    #           LA at home
    #           Vacuum pump off
    #           Phone flipped over
    #           Pneumatics raised

    # Inputs:   factory (IP address)

    # Outputs:  True if successful (done command)
    #           False if unsuccessful (kill command)

    # Arm movements list
    moveHome = ['xh', 'yh', 'zh', 'ah', 'bh']
    moveFlip0 = ['x0', 'y0', 'z0', 'a0', 'b0']
    moveFlip1 = ['x1', 'y1', 'z1', 'a1', 'b1']
    moveFlip2 = ['x2', 'y2', 'z2', 'a2', 'b2']

    # Turn on vacuum
    vpump(True, factory)

    # Move arm to top of crack platform
##    dornaMove(moveFlip0)

    # Slowly lower effector until arm transducer reads vacuum
    if slowApproachArm(factory) == False:
        return False
    
    # Slide phone off of platform
##    if LAMove(moveFlip0, factory) == False:
##        return False

    # Flip phone over and 
##    dornaMove(moveFlip1)

    # Check phone hasn't been accidentally released
    if ArmTrans(factory) == False:
        return False

    # Slide phone onto platform
    if LAHome(factory) == False:
        return False

    # Release vacuum
    vpump(False, factory)

    # Check phone has been released
    if ArmTrans(factory) == True:
        return False

    # Move arm out of the way
##    dornaMove(moveFlip2)

    # Move arm to its home orientation
##    dornaMove(moveHome)

    return True

def toCarriage(sequence, factory):

    # Purpose:  Lift phone from either top or bottom of crack platform
    #           Move phone to carriage
    #           Attach to carriage and release phone
    #           Move out of the way and trigger drop
    
    # Assume:   Arm starts in home orientation
    #           LA starts at home
    #           Vacuum pump off
    #           Phone placed on crack platform (either side face up)
    #           Sequence pregenerated in C#
    
    # End:      Arm in home orientation
    #           LA at home
    #           Vacuum pump off
    #           Phone dropped and in drop box
    #           Pneumatics raised

    # Inputs:   sequence = [x, y, z, a, b, ud]
    #           factory (IP address)

    # Outputs:  True if successful (done command)
    #           False if unsuccessful (kill command)

    # Arm movements list
    moveHome = ['xh', 'yh', 'zh', 'ah', 'bh']
    moveCarrU = ['xu', 'yu', 'zu', 'au', 'bu']
    moveCarrD = ['xd', 'yd', 'zd', 'ad', 'bd']
    moveCarr0 = ['x0', 'y0', 'z0', 'a0', 'b0']
    moveCarr1 = ['x1', 'y1', 'z1', 'a1', 'b1']

    # Check whether to grab top of phone or bottom
    if ud == 1:
        # Move arm above phone
##        dornaMove(moveCarrU)

        # Turn on vacuum
        vpump(True, factory)
        
        # Slow approach until vacuum
        if slowApproachArm(factory) == False:
            return False
        
    else:
        # Move arm to underside of phone
##        dornaMove(moveCarrD)

        # Turn on vacuum
        vpump(True, factory)
        
        # Check phone has been grabbed
        if ArmTrans(factory) == False:
            return False

    # Slide phone off of platform
    if LAMove(sequence,factory) == False:
        return False

    # Move arm up to carriage
##    dornaMove(moveCarr0)
##    dornaMove(moveCarr1)
    dornaMove(sequence)

    # Check phone hasn't been accidentally released
    if ArmTrans(factory) == False:
        return False
    
    # Slow approach at different angles until carriage vacuum achieved
##    if sequence[3] == "aBottom":
##        slowApproachCar("up", factory)
##    else if sequence[3] == "aSide":
##        slowApproachCar("away", factory)
##    else if sequence[3] == "a45":
##        slowApproachCar("45", factory)

    # Check phone is still on arm
    if ArmTrans(factory) == False:
        return False
        
    # Release vacuum
    vpump(False,factory)

    # Check phone has been released
    if ArmTrans(factory) == True:
        return False

    # Move arm to home position
    dornaMove(moveHome)

    # Move LA home and out of the way of carriage
    if LAHome(factory) == False:
        return False
    
    # Trigger drop
    drop(factory)
    
    return True

def liftToCrack(location, factory):

    # Purpose:  Pick up phone from drop box
    #           Place it on crack platform
    
    # Assume:   Arm starts in home orientation
    #           LA starts at home
    #           Vacuum pump off
    #           Phone dropped and in drop box
    #           Location found in C#

    # End:      Arm in home orientation
    #           LA at home
    #           Vacuum pump off
    #           Phone placed on crack platform (either side face up)
    #           Pneumatics raised

    # Inputs:   location = [x, y, z, a, b]
    #           factory (IP address)

    # Outputs:  True if successful (done command)
    #           False if unsuccessful (kill command)

    # Arm movements list
    moveHome = ['xh', 'yh', 'zh', 'ah', 'bh']
    moveCrack0 = ['x0', 'y0', 'z0', 'a0', 'b0']
    moveCrack1 = ['x1', 'y1', 'z1', 'a1', 'b1']

    # Move LA out to position
    if LAMove(location, factory) == False:
        return False

    # Move arm to above the phone
    dornaMove(location)

    # Turn on vacuum
    vpump(True,factory)
    
    # Lower pneumatics
    raiseLower(False,factory)
    
    # Slowly lower effector until arm transducer reads vacuum
    if slowApproachArm(factory) == False:
        return False
    
    # Raise pneumatics
    raiseLower(True,factory)

    # Check phone hasn't been released
    if ArmTrans(factory) == False:
        return False

    # Align arm with bottom of crack platform
##    dornaMove(moveCrack0)

    # Slide phone onto platform
    if LAHome(factory) == False:
        return False

    # Check phone hasn't been accidentally released
    if ArmTrans(factory) == False:
        return False

    # Release vacuum
    vpump(False,factory)

    # Check phone has been released
    if ArmTrans(factory) == True:
        return False

    # Move arm out of the way
##    dornaMove(moveCrack1)

    # Move arm to its home orientation
##    dornaMove(moveHome)
    
    return True

def slowApproachArm(factory):

    # Purpose:  Slowly lower effector until vacuum is achieved
    #           If no vacuum found in 5 sec send kill command

    # Assume:   None

    # End:      Phone attached to effector

    # Input:    factory (IP address)

    # Output:   True = found vacuum within 5 sec
    #           False = no vacuum within 5 sec

    # Get start time
    tStart = time.time()

    # Move arm down 1 mm at a time until vacuum is achieved
    # If vacuum isn't achieved after 5 sec return false
    while ArmTrans(factory) == False:
        dornaMoveRel([0, 0, -1, 0, 0])
        tCurr = time.time()
        if tCurr - tStart > 5:
            return False
    return True

def slowApproachCar(direction, factory):

    # Purpose:  Slowly move effector in specified direction until vacuum is achieved
    #           If no vacuum found in 5 sec send kill command

    # Assume:   None

    # End:      Phone attached to carriage and effector

    # Input:    factory (IP address)
    #           direction (which direction to slowly move in)

    # Output:   True = found vacuum within 5 sec
    #           False = no vacuum within 5 sec

    # Set initial movement increments to 0
    x = 0;
    z = 0;
    
    # Get start time
    tStart = time.time()

    # Set direction of slow movement based on direction
    if direction == "up":
        z = 1
    else if direction == "away":
        x = 1
    else if direction == "45":
        x = 0.5
        z = 0.5

    # Move arm in set direction slowly until vacuum is achieved
    # If vacuum isn't achieved after 5 sec return false
    while CarTrans(factory) == False:
            dornaMoveRel([x, 0, z, 0, 0])
            tCurr = time.time()
            if tCurr - tStart > 5:
                return False
    return True
        
def dornaMove(move):

    # Purpose:  Move dorna to the move location
    #           Wait until movement has been carried out

    # Assume:   None

    # End:      Arm moved to input location

    # Input:    move = [x, y, z, a, b]

    # Output:   None

    # Connect to arm and print current position
    print(robot.connect())
    print(robot.position(space = "xyz"))

    # Send absolute move command to arm
    robot.play({"command": "move", "prm": {"path": "joint", "movement": 0, "speed": 500.0, "x": move[0], "z": move[2], "a": move[3], "b": move[4]}})

    # Wait until command is completed
    state = "1"
    while state == "1":
        current = robot.device()
        state = current[-2]

    # Print current position
    print(robot.position(space = "xyz"))

def dornaMoveRel(move):

    # Purpose:  Move dorna relative to current location
    #           Wait until movement has been carried out

    # Assume:   None

    # End:      Arm moved by the relative amount

    # Input:    move = [x, y, z, a, b]

    # Output:   None

    # Connect to arm and print current position
    print(robot.connect())
    print(robot.position(space = "xyz"))

    # Send relative move command to arm
    robot.play({"command": "move", "prm": {"path": "joint", "movement": 1, "speed": 500.0, "x": move[0], "z": move[2], "a": move[3], "b": move[4]}})

    # Wait until command is completed
    check = 0
    state = "1"
    while check == 0:
        current = robot.device()
        print(current)
        state = current[-2]
        print(state)
        if state == "0":
            check = 1

    # Print current position
    print(robot.position(space = "xyz"))

def LAMove(move, factory):

    # Purpose:  Move LA to the move location
    #           Wait until movement has been carried out
    #           Can only move LA away from home

    # Assume:   None

    # End:      LA moved to input location

    # Input:    move = [x, y, z, a, b]
    #           factory (IP address)

    # Output:   True = moved to location
    #           False = couldn't move to location

    # Print y destination
    y = move[1]
    print('LA moving to ' + str(y))

    # Set Pi pin for direction
    DIR = gpiozero.DigitalOutputDevice(20, active_high=False, pin_factory=factory)
    DIR.off()   # Sets direction to away from home

    # Conversion constants
    d2p = 400 / 5       # 400 pulses per rev / 5mm lead
    p2t = 1 / 1600      # frequency is 1600Hz

    # Calculate time to sleep and print it
    # If time calculated is too great return False
    t = abs(y)* d2p * p2t
    print('Time: ' + str(t))
    if t > 16:
        return False

    # Start step pulse and sleep for calculated time
    STEP = gpiozero.PWMOutputDevice(21, active_high=False, initial_value=0.5, frequency=1600, pin_factory=factory)
    sleep(t)

    # End step pulse
    STEP.off()

    # Print destination reached
    print('LA moved to ' + str(y))
    return True

def vpump(state,factory):

    # Purpose:  Turn on and off vacuum
    #           Close vacuum solenoid when vacuum pump is on
    #           Open vacuum solenoid when vacuum pump is off (vent vacuum)

    # Assume:   None

    # End:      Vacuum turned on or off

    # Input:    Desired state (on = true, off = false)
    #           factory (IP address)

    # Output:   None

    # Set Pi pins for controlling vacuum pump and vacuum solenoid
    vPump = gpiozero.DigitalOutputDevice(19,active_high=True,pin_factory=factory) 
    vSol = gpiozero.DigitalOutputDevice(15,active_high=True,pin_factory=factory)

    # If True enable vacuum else disable vacuum
    if state == True:
        vPump.on()
        vSol.off()
    else:
        vPump.off()
        vSol.on()

def raiseLower(state,factory):

    # Purpose:  Raise or lower pneumatic lift

    # Assume:   None

    # End:      Pneumatics raised or lowered

    # Input:    Desired state (raise = true, lower = false)
    #           factory (IP address)

    # Output:   None

    # Set Pi pins for controlling two pneumatic solenoids
    Sol1 = gpiozero.DigitalOutputDevice(17,active_high=True,pin_factory=factory) 
    Sol2 = gpiozero.DigitalOutputDevice(27,active_high=True,pin_factory=factory)

    # If True raise pneumatic actuators else lower them
    if state == True:
        Sol1.on()
        Sol2.off()
    else:
        Sol1.off()
        Sol2.on()

def drop(factory):

    # Purpose:  Trigger Heina drop

    # Assume:   Phone mounted on carriage in correct orientation

    # End:      Phone dropped and in drop box

    # Input:    factory (IP address)

    # Output:   None

    # Set Pi pin for triggering drop
    drop = gpiozero.DigitalOutputDevice(26,active_high=True,pin_factory=factory)

    # Trigger drop then wait and reset
    drop.on()
    sleep(1)
    drop.off()

def ArmTrans(factory):
    
    # Purpose:  Check end effector for vacuum
    #           If no vacuum return false
    #           If vacuum return true

    # Assume:   None

    # End:      Outputs if the phone is on the arm

    # Input:    factory (IP address)

    # Output:   True = vacuum
    #           False = no vacuum

    # Set Pi pin for reading arm vacuum transducer voltage digitally
    ArmTrans=gpiozero.DigitalInputDevice(8,pull_up=True,bounce_time=None,pin_factory=factory)

    # Begin reading voltage
    ArmTrans.on()

    # If vacuum return True else return False
    if ArmTrans.value == 0:
        return True
    else:
        return False

def CarTrans(factory):
    
    # Purpose:  Check carriage for vacuum
    #           If no vacuum return false
    #           If vacuum return true

    # Assume:   None

    # End:      Outputs if the phone is on the carriage

    # Input:    factory (IP address)

    # Output:   True = vacuum
    #           False = no vacuum

    # Set Pi pin for reading carriage vacuum transducer voltage digitally
    CarTrans=gpiozero.DigitalInputDevice(7,pull_up=True,bounce_time=None,pin_factory=factory)

    # Begin reading voltage
    CarTrans.on()

    # If vacuum return True else return False
    if CarTrans.value == 0:
        return True
    else:
        return False

def LAHome(factory):

    # Purpose:  Send LA to home position

    # Assume:   None

    # End:      LA located in its home position

    # Input:    factory (IP address)

    # Output:   True = reached home
    #           False = failed to reach home

    print('LA going home')

    # Set Pi pins for reading limit switch, controlling direction, and outputting a step pulse
    HOME = gpiozero.Button(14, pull_up=False, pin_factory=factory)
    DIR =  gpiozero.DigitalOutputDevice(20, active_high=False, pin_factory=factory)
    DIR.on()    # Sets direction towards home
    STEP = gpiozero.PWMOutputDevice(21, active_high=False, initial_value=0.5, frequency=1600, pin_factory=factory)

    tStart = time.time()
    
    # Wait until limit switch is triggered then disable step pulse and return True
    # If limit switch isn't triggered for 30 sec disable step pulse and return False
    while HOME.value == 1:
        STEP.off()
        DIR.off()
        tCurr = time.time()
        if tCurr - tStart > 16:
            STEP.off()
            DIR.off()
            return False
    print('LA home')
    return True
