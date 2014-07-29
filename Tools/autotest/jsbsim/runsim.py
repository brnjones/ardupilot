#!/usr/bin/env python
# run a jsbsim model as a child process

#TODO (Jacob) re-introduce wind modelling
#TODO (Jacob) re-introduce flight gear visualization
#TODO (Jacob) Figure out what to do about altitude spawn
import os, time, sys, subprocess, socket, threading, struct, Queue, telnetlib, re, math, errno
from pymavlink import fgFDM

import pdb

_kRecvTimeout = 1.0 # Seconds
_kWriteTimeout = 0.1 # Seconds
_kConsoleTimeOut = 0.001 # Seconds

def setup_scripts(autotest_path, ports, home, vehicle, simrate):
    home_list = home.split(',')
    if len(home_list) != 4:
        raise ValueError("home should be lat,lng,alt,hdg - '%s'" % home)
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
                                    'FGOUTPORT'  : int(port),
                                    'SIMRATE'    : int(simrate)}
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
    def __init__(self, sitl_in, jsb_in, vehicle, out_q):
        super(SitlInThread, self).__init__()
        # Create the input port to this process
        self._sitl_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sitl_in.bind(interpret_address(sitl_in))
        self._sitl_in.setblocking(0)
        self._sitl_in.settimeout(_kRecvTimeout)
        # Connect to JSBSim's input
        host, port = interpret_address(jsb_in)
        self._jsb_con = telnetlib.Telnet(host, port, _kConsoleTimeOut)
        # Other Stuff
        self._vehicle = vehicle
        self._stop = threading.Event()
        self._out_q = out_q
        self._start_time = time.time()
        self.debug_state = ''


    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.is_set()

    def run(self):
        try:
            self._jsb_con.write('info\n')
            time.sleep(0.2)
            self._out_q.put(self._dump_from_jsb_console())
            self._jsb_con.write('resume\n')
            time.sleep(0.2)
            self._out_q.put(self._dump_from_jsb_console())
            self._run_loop()
        except:
            self.stop()
            self._out_q.put_nowait("SITL in, Unexpected error:" + 
                                   str(sys.exc_info()[0]))
            raise
            
    def _run_loop(self):
        while not self._stop.is_set():
            jsb_message = self._dump_from_jsb_console()
            if jsb_message != '':
                self._out_q.put(jsb_message)
            try:
                buf = self._sitl_in.recv(28)
            except socket.timeout:
                continue
            control = list(struct.unpack('<14H', buf))
            pwm_out = control[:11]
            self._relay_to_jsb(pwm_out)
        self.stop()

    def _relay_to_jsb(self, pwm):
        if self._vehicle == 'Rascal':
            # TODO
            pass
        elif self._vehicle == 'Rascucopter':
            if len(pwm) < 8:
                raise ValueError('Pwm length must at least 8')
            throttles = [0.0]*5
            for i in range(0, 5):
                throttles[i] = (pwm[i]-1080)/1000.0
                if throttles[i] < 0.0:
                    throttles[i] = 0.0
                elif throttles[i] > 1.0:
                    throttles[i] = 1.0
            aileron = (pwm[5]-1500)/500.0
            elevator =(pwm[6]-1500)/500.0
            rudder = (pwm[7]-1500)/500.0
            sim_time = time.time() -self._start_time
            self.debug_state = 'Thr1: %s\tThr2: %s\tThr3: %s\tThr4: %s\t' % (
                throttles[0], 
                throttles[1],
                throttles[2],
                throttles[3])
            armed = False
            for throt in throttles:
                if throt > 0.0:
                    armed = True
            self._set_jsb_console('fcs/ne_motor', throttles[0])
            self._set_jsb_console('fcs/sw_motor', throttles[1])
            self._set_jsb_console('fcs/nw_motor', throttles[2])
            self._set_jsb_console('fcs/se_motor', throttles[3])
            # TODO (Jacob) Kill this
            # if armed:
            #     self._set_jsb_console('aero/roll_moment_probe', 0.2*math.sin(0.5*sim_time))
            # else:
            #     self._set_jsb_console('aero/roll_moment_probe', 0.0)
        else:
            raise ValueError("Vehicle does not match predefined types")

    def _set_jsb_console(self, variable, value):
        self._jsb_con.write('set %s %s\r\n' % (variable, value))
        return_str = self._jsb_con.read_very_eager()

    def _dump_from_jsb_console(self):
        buf = self._jsb_con.read_very_eager()
        buf = buf.replace('JSBSim> ', '')
        return buf


class SitlOutThread(threading.Thread):
    def __init__(self, sitl_out, jsb_out, out_rate, out_q):
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
        self._out_q = out_q
        self._stop = threading.Event()
        self._fdm = fgFDM.fgFDM()
        self._out_rate = out_rate
        self._last_out = 0
        self.debug_state = ''

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.is_set()

    def run(self):
        try:
            self._run_loop()
        except:
            self.stop()
            self._out_q.put_nowait("SITL out, Unexpected error:" + 
                                   str(sys.exc_info()[0]))
            raise
 
    def _run_loop(self):
        while not self._stop.is_set():
            try:
                buf = self._jsb_out.recv(self._fdm.packet_size())
            except socket.timeout:
                continue
            now = time.time()
            if (now - self._last_out) > (1.0/self._out_rate):
                self._last_out = now
                self._relay_to_sitl_out(buf)
        self.stop()
        
    def _relay_to_sitl_out(self, buf):
        self._fdm.parse(buf)
        fdm = self._fdm
        self.debug_state = 'Alt: %s\t, v_down: %s, a_down: %s\nRoll: %s\t Pitch: %s Yaw: %s' % (
                             fdm.get('altitude', units='meters'),
                             fdm.get('v_down', units='mps'),
                             fdm.get('A_Z_pilot', units='mpss'),
                             fdm.get('phi', units='degrees'),
                             fdm.get('theta', units='degrees'),
                             fdm.get('psi', units='degrees'))

        out_buf  = struct.pack('<17dI',
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
            self._sitl_out.send(out_buf)
        except socket.error as e:
            if e.errno not in [ errno.ECONNREFUSED ]:
                raise


def dump_queue(queue):
    out = list()
    while True:
        try:
            message = queue.get_nowait()
            out.append(message)
        except Queue.Empty:
            break
    return out


def main(args):    
    ports = {'sitl_in' : args.simin,
             'sitl_out': args.simout,
             'sitl_out_fg' : args.fgout,
             'jsb_in' : args.jsbin,
             'jsb_out' : args.jsbout}
    autotest_path = os.path.realpath(os.path.join(__file__,'..','..'))
    setup_scripts(autotest_path, ports, args.home, args.vehicle, args.simrate)
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
    log_q = Queue.Queue()
    jsb_proc = None
    sitl_in_thread = None
    sitl_out_thread = None
    try:
        jsb_proc = subprocess.Popen(jsb_command.strip().split(' '))
        time.sleep(8)
        sitl_in_thread = SitlInThread(ports['sitl_in'], ports['jsb_in'], args.vehicle, log_q)
        sitl_out_thread = SitlOutThread(ports['sitl_out'], ports['jsb_out'], args.outrate, log_q)
        sitl_in_thread.start()
        sitl_out_thread.start()
        #TODO Wait for processes to finish
        while True:
            sub_proc_output = dump_queue(log_q)
            if sub_proc_output:
                for str_ in sub_proc_output:
                    print(str_)
            print(sitl_out_thread.debug_state)
            print(sitl_in_thread.debug_state)
            if sitl_in_thread.stopped():
                print("Something killed sitl_in_thread")
                break
            if sitl_out_thread.stopped():
                print("Something killed sitl_out_thread")
                break
            if jsb_proc.poll() is not None:
                print("Something killed JSBSim")
                break
            time.sleep(1)
        #TODO Setup Flightgear output
    except:
        print("Unexpected error:" + str(sys.exc_info()))
    finally:
        print("Shutting down sim")
        if jsb_proc and jsb_proc.pid is not None:
            print("Killing JSBSim")
            if jsb_proc.poll() is None:
                jsb_proc.terminate()
            jsb_proc.wait()
            print("JSBSim is dead X^X. Rejoice >:-)")
        if sitl_in_thread and not sitl_in_thread.stopped():
            print("Killing SITL Input Thread")
            sitl_in_thread.stop()
            sub_proc_output = dump_queue(log_q)
            if sub_proc_output:
                print(sub_proc_output)
            sitl_in_thread.join()
            print("SITL input thread dead")
        if sitl_out_thread and not sitl_out_thread.stopped():
            print("Killing SITL Output Thread")
            sitl_out_thread.stop()
            sub_proc_output = dump_queue(log_q)
            if sub_proc_output:
                print(sub_proc_output)
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
    parser.add_argument('--outrate', help='Rate the simulation outputs data', type=int, default=100)
    args = parser.parse_args()

    main(args)
