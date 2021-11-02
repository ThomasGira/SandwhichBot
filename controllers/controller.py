import sys
import tempfile
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')


IKPY_MAX_ITERATIONS = 4

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))
armChain = Chain.from_urdf_file(filename)
for i in [0, 6]:
    armChain.active_links_mask[0] = False

# Initialize the arm motors and encoders.
motors = []
for link in armChain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)
        
# Loop 1: Draw a circle on the paper sheet.
print('Draw a circle on the paper sheet...')
while supervisor.step(timeStep) != -1:
    t = supervisor.getTime()

    # Use the circle equation relatively to the arm base as an input of the IK algorithm.
    x = 0.25 * math.cos(t) + 1.1
    y = 0.25 * math.sin(t) - 0.95
    z = 0.05

    # Call "ikpy" to compute the inverse kinematics of the arm.
    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
    ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

    # Actuate the 3 first arm motors with the IK results.
    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])
    # Keep the hand orientation down.
    motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
    # Keep the hand orientation perpendicular.
    motors[5].setPosition(ikResults[1])

    # Conditions to start/stop drawing and leave this loop.
    if supervisor.getTime() > 2 * math.pi + 1.5:
        break
    elif supervisor.getTime() > 1.5:
        # Note: start to draw at 1.5 second to be sure the arm is well located.
        supervisor.getDevice('pen').write(True)