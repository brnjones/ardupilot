#!/usr/bin/env python
# run a jsbsim model as a child process

import sys, os, pexpect, socket
import math, time, select, struct, signal, errno

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'pysim'))

import util, atexit, fdpexpect
from pymavlink import fgFDM

class control_state(object):
    def __init__(self):
        self.aileron = 0
        self.elevator = 0
        self.throttle = 0
        self.rudder = 0
        self.ground_height = 0
        self.throttle0 = 0
        self.throttle1 = 0
        self.throttle2 = 0
        self.throttle3 = 0

class log_state(object):
    def __init__(self):
        self.SITL_input = ""
        self.SITL_output= ""

sitl_state = control_state()
sitl_log = log_state()

def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

def jsb_set(variable, value):
    '''set a JSBSim variable'''
    global jsb_console
    jsb_console.send('set %s %s\r\n' % (variable, value))

def setup_template(home, vehicle):
    global opts
    v = home.split(',')
    if len(v) != 4:
        print("home should be lat,lng,alt,hdg - '%s'" % home)
        sys.exit(1)
    latitude = float(v[0])
    longitude = float(v[1])
    altitude = float(v[2])
    heading = float(v[3])
    sitl_state.ground_height = altitude
    template = os.path.join('aircraft', vehicle, 'reset_template.xml')
    reset = os.path.join('aircraft', vehicle, 'reset.xml')
    xml = open(template).read() % { 'LATITUDE'  : str(latitude),
                                    'LONGITUDE' : str(longitude),
                                    'HEADING'   : str(heading) }
    open(reset, mode='w').write(xml)
    print("Wrote %s" % reset)

    baseport = int(opts.simout.split(':')[1])

    template = os.path.join('jsbsim', 'fgout_template.xml')
    out      = os.path.join('jsbsim', 'fgout.xml')
    xml = open(template).read() % { 'FGOUTPORT'  : str(baseport+3) }
    open(out, mode='w').write(xml)
    print("Wrote %s" % out)

    template = os.path.join('jsbsim', vehicle, 'test_template.xml')
    out      = os.path.join('jsbsim', vehicle, 'test.xml')
    xml = open(template).read() % { 'JSBCONSOLEPORT'  : str(baseport+4) }
    open(out, mode='w').write(xml)
    print("Wrote %s" % out)
    

def process_sitl_input(buf, vehicle):
    '''process control changes from SITL sim'''
    control = list(struct.unpack('<14H', buf))
    pwm = control[:11]
    (speed, direction, turbulance) = control[11:]

    global wind, frame_count
    wind.speed      = speed*0.01
    wind.direction  = direction*0.01
    wind.turbulance = turbulance*0.01
    if vehicle == "Rascal":
        # Map inputs from 1000-2000 to 0-1 or -1 to 1
        aileron  = (pwm[0]-1500)/500.0
        elevator = (pwm[1]-1500)/500.0
        throttle = (pwm[2]-1000)/1000.0
        if opts.revthr:
            throttle = 1.0 - throttle
            rudder   = (pwm[3]-1500)/500.0
            
        if opts.elevon:
            # fake an elevon plane
            ch1 = aileron
            ch2 = elevator
            aileron  = (ch2-ch1)/2.0
            # the minus does away with the need for RC2_REV=-1
            elevator = -(ch2+ch1)/2.0

        if opts.vtail:
            # fake an elevon plane
            ch1 = elevator
            ch2 = rudder
            # this matches VTAIL_OUTPUT==2
            elevator = (ch2-ch1)/2.0
            rudder   = (ch2+ch1)/2.0
        
        if aileron != sitl_state.aileron:
            jsb_set('fcs/aileron-cmd-norm', aileron)
            sitl_state.aileron = aileron
        if elevator != sitl_state.elevator:
            jsb_set('fcs/elevator-cmd-norm', elevator)
            sitl_state.elevator = elevator
        if rudder != sitl_state.rudder:
            jsb_set('fcs/rudder-cmd-norm', rudder)
            sitl_state.rudder = rudder
        if throttle != sitl_state.throttle:
            jsb_set('fcs/throttle-cmd-norm', throttle)
            sitl_state.throttle = throttle
    elif vehicle == "Rascucopter":
        throttle0 = (pwm[0]-1080)/1000.0
        throttle1 = (pwm[1]-1080)/1000.0
        throttle2 = (pwm[2]-1080)/1000.0
        throttle3 = (pwm[3]-1080)/1000.0
        throttle4 = (pwm[4]-1080)/1000.0
        aileron = (pwm[5]-1500)/500.0
        elevator =(pwm[6]-1500)/500.0
        rudder = (pwm[7]-1500)/500.0
        throttle0 = lock_to_range(throttle0, 0.0, 1.0)
        throttle1 = lock_to_range(throttle1, 0.0, 1.0)
        throttle2 = lock_to_range(throttle2, 0.0, 1.0)
        throttle3 = lock_to_range(throttle3, 0.0, 1.0)
        throttle4 = lock_to_range(throttle4, 0.0, 1.0)
        sitl_log.SITL_input = "Outs: %s" % str([
            throttle0,
            throttle1,
            throttle2,
            throttle3,
            throttle4])
        if throttle0 != sitl_state.throttle0:
            jsb_set('fcs/ne_motor', throttle0)
        if throttle1 != sitl_state.throttle1:
            jsb_set('fcs/sw_motor', throttle1)
        if throttle2 != sitl_state.throttle2:
            jsb_set('fcs/nw_motor', throttle2)
        if throttle3 != sitl_state.throttle3:
            jsb_set('fcs/se_motor', throttle3)
        #jsb_set('fcs/throttle-cmd-norm', throttle4)
        #jsb_set('fcs/aileron-cmd-norm', aileron)
        #jsb_set('fcs/elevator-cmd-norm', elevator)
        #jsb_set('fcs/rudder-cmd-norm', rudder)
    else:
        raise ValueError("Vehicle does not match predefined types")

def lock_to_range(value, min_, max_):
    if value < min_:
        value = min_
    elif value > max_:
        value = max_
    return value


def update_wind(wind):
    '''update wind simulation'''
    (speed, direction) = wind.current()
    jsb_set('atmosphere/psiw-rad', math.radians(direction))
    jsb_set('atmosphere/wind-mag-fps', speed/0.3048)
    

def process_jsb_input(buf):
    '''process FG FDM input from JSBSim'''
    global fdm, fg_out, sim_out, frame_count
    fdm.parse(buf)
    if fg_out:
        try:
            agl = fdm.get('agl', units='meters')
            fdm.set('altitude', agl+sitl_state.ground_height, units='meters')
            fdm.set('rpm', sitl_state.throttle*1000)
            fg_out.send(fdm.pack())
        except socket.error as e:
            if e.errno not in [ errno.ECONNREFUSED ]:
                raise
    simlist = [fdm.get('altitude', units='meters'),
               fdm.get('A_Z_pilot', units='mpss'),
               fdm.get('phi', units='degrees'),
               fdm.get('theta', units='degrees')]

    out_string = ""
    for val in simlist:
        out_string += "%.3f\t" % val
    out_string += "\n"
    sitl_log.SITL_output = out_string
    simbuf = struct.pack('<17dI',
                         fdm.get('latitude', units='degrees'),
                         fdm.get('longitude', units='degrees'),
                         fdm.get('altitude', units='meters'),
                         fdm.get('psi', units='degrees'),
                         fdm.get('v_north', units='mps'),
                         fdm.get('v_east', units='mps'),
                         fdm.get('v_down', units='mps'),
                         fdm.get('A_X_pilot', units='mpss'),
                         fdm.get('A_Y_pilot', units='mpss'),
                         fdm.get('A_Z_pilot', units='mpss'),
                         fdm.get('phidot', units='dps'),
                         fdm.get('thetadot', units='dps'),
                         fdm.get('psidot', units='dps'),
                         fdm.get('phi', units='degrees'),
                         fdm.get('theta', units='degrees'),
                         fdm.get('psi', units='degrees'),
                         fdm.get('vcas', units='mps'),
                         0x4c56414f)
    try:
        sim_out.send(simbuf)
    except socket.error as e:
        if e.errno not in [ errno.ECONNREFUSED ]:
            raise



##################
# main program
from optparse import OptionParser
parser = OptionParser("runsim.py [options]")
parser.add_option("--simout",  help="SITL output (IP:port)",         default="127.0.0.1:5501")
parser.add_option("--simin",   help="SITL input (IP:port)",          default="127.0.0.1:5502")
parser.add_option("--fgout",   help="FG display output (IP:port)",   default="127.0.0.1:5503")
parser.add_option("--jsbin",   help="jsb input (IP:port)",   default="127.0.0.1:5504")
parser.add_option("--jsbout",   help="jsb output (IP:port)",   default="127.0.0.1:5505")
parser.add_option("--home",    type='string', help="home lat,lng,alt,hdg (required)")
parser.add_option("--vehicle", type='string', help="(Rascal, Rascucopter)", default="Rascal")
parser.add_option("--script",  type='string', help='jsbsim model script', default='test.xml')
parser.add_option("--options", type='string', help='jsbsim startup options')
parser.add_option("--elevon", action='store_true', default=False, help='assume elevon input')
parser.add_option("--revthr", action='store_true', default=False, help='reverse throttle')
parser.add_option("--vtail", action='store_true', default=False, help='assume vtail input')
parser.add_option("--wind", dest="wind", help="Simulate wind (speed,direction,turbulance)", default='0,0,0')

(opts, args) = parser.parse_args()

for m in [ 'home', 'script' ]:
    if not opts.__dict__[m]:
        print("Missing required option '%s'" % m)
        parser.print_help()
        sys.exit(1)

os.chdir(util.reltopdir('Tools/autotest'))

# kill off child when we exit
atexit.register(util.pexpect_close_all)

setup_template(opts.home, opts.vehicle)

script = os.path.join('jsbsim', opts.vehicle, opts.script)
# start child
cmd = "JSBSim --realtime --suspend --nice --simulation-rate=100 --logdirectivefile=jsbsim/fgout.xml --script=%s" % script
if opts.options:
    cmd += ' %s' % opts.options

print("Running Command: \n\"%s\"" % cmd)
jsb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
jsb.delaybeforesend = 0
util.pexpect_autoclose(jsb)
i = jsb.expect(["Successfully bound to socket for input on port (\d+)",
                "Could not bind to socket for input"])
if i == 1:
    print("Failed to start JSBSim - is another copy running?")
    sys.exit(1)
jsb_out_address = interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
jsb.expect("Creating UDP socket on port (\d+)")
jsb_in_address = interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
jsb.expect("Successfully connected to socket for output")
jsb.expect("JSBSim Execution beginning")

# setup output to jsbsim
print("JSBSim console on %s" % str(jsb_out_address))
jsb_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
jsb_out.connect(jsb_out_address)
jsb_console = fdpexpect.fdspawn(jsb_out.fileno(), logfile=sys.stdout)
jsb_console.delaybeforesend = 0

# setup input from jsbsim
print("JSBSim FG FDM input on %s" % str(jsb_in_address))
jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
jsb_in.bind(jsb_in_address)
jsb_in.setblocking(0)

# setup input from SITL sim
sim_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_in.bind(interpret_address(opts.simin))
sim_in.setblocking(0)

# setup output to SITL sim
sim_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_out.connect(interpret_address(opts.simout))
sim_out.setblocking(0)

# setup possible output to FlightGear for display
fg_out = None
if opts.fgout:
    fg_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fg_out.connect(interpret_address(opts.fgout))


# setup wind generator
wind = util.Wind(opts.wind)

fdm = fgFDM.fgFDM()

jsb_console.send('info\n')
jsb_console.send('resume\n')
#jsb.expect(["trim computation time","Trim Results"])
time.sleep(3.0)
jsb_console.logfile = None

print("Simulator ready to fly")

def main_loop():
    '''run main loop'''
    tnow = time.time()
    last_report = tnow
    last_sim_input = tnow
    last_wind_update = tnow
    global frame_count
    frame_count = 0
    paused = False

    while True:
        rin = [jsb_in.fileno(), sim_in.fileno(), jsb_console.fileno(), jsb.fileno()]
        try:
            (rin, win, xin) = select.select(rin, [], [], 1.0)
        except select.error:
            util.check_parent()
            continue
        tnow = time.time()
        
        if jsb_in.fileno() in rin:
            buf = jsb_in.recv(fdm.packet_size())
            process_jsb_input(buf)
            frame_count += 1

        if sim_in.fileno() in rin:
            simbuf = sim_in.recv(28)
            process_sitl_input(simbuf, opts.vehicle)
            last_sim_input = tnow

        # show any jsbsim console output
        if jsb_console.fileno() in rin:
            util.pexpect_drain(jsb_console)
        if jsb.fileno() in rin:
            util.pexpect_drain(jsb)

        if tnow - last_sim_input > 1.0:
            if not paused:
                print("PAUSING SIMULATION")
                paused = True
                jsb_console.send('hold\n')
        else:
            if paused:
                print("RESUMING SIMULATION")
                paused = False
                jsb_console.send('resume\n')

        # only simulate wind above 5 meters, to prevent crashes while
        # waiting for takeoff
        if tnow - last_wind_update > 0.1:
            update_wind(wind)
            last_wind_update = tnow

        if tnow - last_report > 0.5:
            print("FPS %u asl=%.3f agl=%.3f roll=%.3f pitch=%.3f a=(%.2f %.2f %.2f)" % (
                frame_count / (time.time() - last_report),
                fdm.get('altitude', units='meters'),
                fdm.get('agl', units='meters'),
                fdm.get('phi', units='degrees'),
                fdm.get('theta', units='degrees'),
                fdm.get('A_X_pilot', units='mpss'),
                fdm.get('A_Y_pilot', units='mpss'),
                fdm.get('A_Z_pilot', units='mpss')))
            print(sitl_log.SITL_input)
            print(sitl_log.SITL_output)
            frame_count = 0
            last_report = time.time()

def exit_handler():
    '''exit the sim'''
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    # JSBSim really doesn't like to die ...
    if getattr(jsb, 'pid', None) is not None:
        os.kill(jsb.pid, signal.SIGKILL)
    jsb_console.send('quit\n')
    jsb.close(force=True)
    util.pexpect_close_all()
    sys.exit(1)

signal.signal(signal.SIGINT, exit_handler)
signal.signal(signal.SIGTERM, exit_handler)

try:
    main_loop()
except Exception as inst:
    print inst
    exit_handler()
    raise
