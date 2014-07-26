#!/usr/bin/env python
# run a jsbsim model as a child process

import os, time, subprocess, socket, threading
from pymavlink import fgFDM

import pdb


_kRecvTimeout = 0.1 # Seconds
_kWriteTimeout = 0.1 # Seconds

def setup_scripts(autotest_path, ports, home, vehicle):
    home_list = home.split(',')
    if len(home_list) != 4:
        print("home should be lat,lng,alt,hdg - '%s'" % home)
        sys.exit(1)
    latitude = float(home_list[0])
    longitude = float(home_list[1])
    altitude = float(home_list[2])
    heading = float(home_list[3])

    # Make the reset script
    template = os.path.join(autotest_path, 'aircraft', vehicle, 'reset_template.xml')
    reset = os.path.join(autotest_path, 'aircraft', vehicle, 'reset.xml')
    xml = open(template).read() % { 'LATITUDE'  : str(latitude),
                                    'LONGITUDE' : str(longitude),
                                    'HEADING'   : str(heading),
                                    'ALTITUDE'  : str(altitude)}
    open(reset, mode='w').write(xml)
    print("Wrote:\n%s" % reset)
    
    # Make the fgout script
    addr, port = ports['jsb_out'].split(':')
    template = os.path.join(autotest_path, 'jsbsim', 'fgout_template.xml')
    out      = os.path.join(autotest_path, 'jsbsim', 'fgout.xml')
    xml = open(template).read() % { 'NAME' : addr,
                                    'FGOUTPORT'  : int(port) }
    open(out, mode='w').write(xml)
    print("Wrote:\n%s" % out)

    # Make the test script
    addr, port = ports['jsb_in'].split(':')
    template = os.path.join(autotest_path, 'jsbsim', vehicle, 'test_template.xml')
    out      = os.path.join(autotest_path, 'jsbsim', vehicle, 'test.xml')
    xml = open(template).read() % { 'JSBCONSOLEPORT'  : str(port) }
    open(out, mode='w').write(xml)
    print("Wrote %s" % out)


def interpret_address(addrstr):
    '''interpret an IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)


class SitlInThread(threading.Thread):
    def __init__(self, sitl_in, jsb_in):
        super(SitlInThread, self).__init__()
        # Create the input port to this process
        self._sitl_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sitl_in.bind(interpret_address(sitl_in))
        self._sitl_in.setblocking(0)
        self._sitl_in.settimeout(_kRecvTimeout)
        # Connect to JSBSim's input
        self._jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._jsb_in.connect(interpret_address(jsb_in))
        self._jsb_in.setblocking(0)
        self._jsb_in.settimeout(_kWriteTimeout)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        while not self._stop:
            try:
                self._sitl_in.recv(28)
            except socket.timeout:
                continue
            # Communication must have succeeded, so procces it.
            # try:
                
            #     self._jsb_in.send(


class SitlOutThread(threading.Thread):
    def __init__(self, sitl_out, jsb_out):
        super( SitlOutThread, self).__init__()
        # Connect to the output of this SITL (MavProxy)
        self._sitl_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sitl_out.connect(interpret_address(sitl_out))
        self._sitl_out.setblocking(0)
        self._sitl_out.settimeout(_kWriteTimeout)
        # Create JSBSim's output port
        self._jsb_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._jsb_out.bind(interpret_address(jsb_out))
        self._jsb_out.setblocking(0)
        self._jsb_out.settimeout(_kRecvTimeout)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        while not self._stop:
            time.sleep(1)


def main(args):    
    ports = {'sitl_in' : args.simin,
             'sitl_out': args.simout,
             'sitl_out_fg' : args.fgout,
             'jsb_in' : args.jsbin,
             'jsb_out' : args.jsbout}
    autotest_path = os.path.realpath(os.path.join(__file__,'..','..'))
    setup_scripts(autotest_path, ports, args.home, args.vehicle)
    # Start JSB sim
    jsb_options = ['JSBSim', 
                   '--realtime', '--suspend', '--nice',
                   '--simulation-rate=%i' % args.simrate,
                   '--logdirectivefile=%s' % os.path.join(autotest_path, 'jsbsim', 'fgout.xml'),
                   '--script=%s' % os.path.join(autotest_path, 'jsbsim', args.vehicle, args.script),
                   args.options]
    jsb_command = ' '.join(jsb_options)
    os.chdir(autotest_path)
    print('Starting JSBSim with Command:\n%s' % jsb_command)
    jsb_proc = None
    sitl_in_thread = None
    sitl_out_thread = None
    try:
        jsb_proc = subprocess.Popen(jsb_command.strip().split(' '))
        sitl_in_thread = SitlInThread(ports['sitl_in'], ports['jsb_in'])
        sitl_out_thread = SitlOutThread(ports['sitl_out'], ports['jsb_out'])
        sitl_in_thread.start()
        sitl_out_thread.start()
        #TODO Wait for processes to finish
        #TODO Setup Flightgear output
        time.sleep(10)
    finally:
        if jsb_proc:
            print("Killing JSBSim")
            jsb_proc.terminate()
            jsb_proc.wait()
            print("JSBSim is dead X^X. Rejoice :)")
        if sitl_in_thread and not sitl_in_thread.stopped():
            print("Killing SITL Input Thread")
            sitl_in_thread.stop()
            sitl_in_thread.join()
            print("SITL input thread dead")
        if sitl_out_thread and not sitl_out_thread.stopped():
            print("Killing SITL Output Thread")
            sitl_out_thread.stop()
            sitl_out_thread.join()
            print("SITL Output thread dead")
    print("Finished")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='runsim.py [options]')
    parser.add_argument('--home',    help='home lat,lng,alt,hdg (required)', required=True)
    parser.add_argument('--script',  help='jsbsim model script', default='test.xml')
    parser.add_argument('--simout',  help='SITL output (IP:port)',         default='127.0.0.1:5501')
    parser.add_argument('--simin',   help='SITL input (IP:port)',          default='127.0.0.1:5502')
    parser.add_argument('--fgout',   help='FG display output (IP:port)',   default='127.0.0.1:5503')
    parser.add_argument("--jsbin",   help="jsb input (IP:port) NOTE:Address is ignored",
                        default="127.0.0.1:5504")
    parser.add_argument("--jsbout",   help="jsb output (IP:port)",   default="127.0.0.1:5505")
    parser.add_argument('--vehicle', help='Model of aircraft to fly', default='Rascal', 
                      choices = ['Rascal', 'Rascucopter'])
    parser.add_argument('--options', help='jsbsim startup options', default='')
    parser.add_argument('--simrate', help='Simulation rate', type=int, default=1000)
    args = parser.parse_args()

    main(args)
