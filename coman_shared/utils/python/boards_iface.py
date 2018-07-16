#! /usr/bin/env python

import sys
import socket
import threading
import time
import datetime
import ctypes
import struct
import json
import traceback
import math
from functools import wraps

#  
import read_xl
import test_lib
import policy_maps
import board_data_type
#import DcMcBoard

lock = threading.Lock()
loop = threading.Event()
    
ZMQ_PUB_PROXY_PORT = 5555
ZMQ_SUB_PROXY_PORT = 5005
ZMQ_REQ_BROKER_PORT = 5559
ZMQ_REP_BROKER_PORT = 5560
ZMQ_REP_PORT = 6969
ZMQ_PULL_PORT = 7777

try : 
    import zmq
    has_zmq = True
    zcontext = zmq.Context(1)
    #
    zpub_sock = zcontext.socket(zmq.PUB)
    zpub_sock.setsockopt(zmq.LINGER, 1)
    #zpub_sock.setsockopt(zmq.HWM, 1000)
    zpub_sock.connect("tcp://*:%d"%ZMQ_SUB_PROXY_PORT)
    
    
    def zmq_pub(bcast_data):
        data_dict = bcast_data.toDict(all_fields=False)
        zpub_sock.send_multipart(['board_%d'%bcast_data._board_id,
                                  json.dumps(data_dict)])
        

except ImportError :
    has_zmq = False

def print_bcast_data(bcast_data):
    data_dict = bcast_data.toDict(all_fields=False)
    print data_dict



MC_packets = read_xl.get_board_packets(r'Motor Controller Interface.xlsx', sheet='Interface')
FT_packets = read_xl.get_board_packets(r'TorqueDSP Interface.xlsx')
DC_packets = read_xl.get_board_packets(r'DC Motor Controller Interface.xlsx', sheet='Interface')

all_packets = dict(
    MC_packets.items() \
    + FT_packets.items() \
    # + DC_packets.items()
)
all_packets_by_cmd = dict(zip([o._cmd for o in all_packets.values()], all_packets.values()))


def timed(f):
    @wraps(f)
    def wrapper(*args, **kwds):
        start = datetime.datetime.now()
        result = f(*args, **kwds)
        elapsed = datetime.datetime.now() - start
        print "%s took %s time to finish" % (f.__name__, elapsed)
        return result
    return wrapper

#@timed
def on_rx_pkt(buff):

    try : pkt = all_packets_by_cmd[ord(buff[2])]
    except KeyError : 
        # No match found
        print '<<<< RX : No match found for', map(lambda x:hex(ord(x)), buff)
        print all_packets_by_cmd.keys()
        return None
    lock.acquire(True)
    rx_pkt = pkt.from_buffer(buff)
    lock.release()
    #print '<<<< RX', rx_pkt.name, map(lambda x:hex(ord(x)), buff)
    return rx_pkt


def board_factory(pkt_data, boards_controller):

    bId   = pkt_data[1]
    bType = pkt_data[0]
    ip    = pkt_data[-4:]
    ip.reverse()
    ip = socket.inet_ntoa(''.join(map(lambda x:chr(int(x)),ip)))

    for b_cls in [McBoard, DcMcBoard.DCMcBoard, FtBoard] :
        if b_cls.board_type == bType :
            return b_cls(bId, bType, ip, boards_controller)
    else :
        raise Exception('Unknown board type %s' % bType )


# ##############################################################################
#
# ##############################################################################

class Board(object):

    @classmethod
    def get_policy_map(cls):
        return cls.policy_map 

    def __init__(self, bId, bType, bAddr, bPort, controller) :

        self.bId = bId
        self.bType = bType
        self.bAddr = bAddr
        self.bPort = bPort
        self.controller = controller
        try : self.conf = self.controller.boards_conf[bType][bId]
        except KeyError : self.conf = self.controller.boards_conf[bType][-1]
        self.policy = self.conf.get('policy', '')
        self.policy_map = self.get_policy_map()

        self.packets = {}
        self.bcast_data_handler = {}

        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_sock.settimeout(1)
        self.tcp_sock.connect((self.bAddr,self.bPort))

        if use_zmq :
            self.bcast_data_handler['zmq_pub'] = zmq_pub

        else : 
            self.bcast_data_handler['stdout'] = print_bcast_data

            
    def parse_policy(self, policy):

        bits = []
        for token in policy.split('|'):
            p = self.policy_map[token]
            bits.append(p['pos'])
        bits.sort(reverse=True)
        pol_num = reduce(lambda x,y: x+y ,map(lambda x: 2**x, bits))
        
        return [pol_num & 0xFF,
                pol_num >> 8 & 0xFF,
                pol_num >> 16 & 0xFF,
                pol_num >> 24 & 0xFF]

    def start(self):

        self.bcast_policy = self.parse_policy(self.policy)
        self.send_tcp_cmd('SET_BCAST_POLICY', cmd_data=self.bcast_policy[:2])
        self.send_tcp_cmd('SET_EXTRA_BCAST_POLICY', cmd_data=self.bcast_policy[2:])
        self.send_tcp_cmd('SET_BCAST_RATE', cmd_data=[self.conf['bcast_rate'],0x1])

    def send_tcp_cmd(self, cmd_name, cmd_data=[]):
        ''' look in proper packets set '''

        try : pkt = self.packets[cmd_name]
        except KeyError : return False 

        buff = pkt.to_buffer(data=cmd_data)
        #print '>>>> TX', pkt.name, map(lambda x:hex(ord(x)), buff)
        self.tcp_sock.send(buff)
        return True

    def recv_tcp_reply(self):

        pkt = None
        try : 
            buff = self.tcp_sock.recv(1024)
            pkt = on_rx_pkt(buff)
            if pkt is None :
                print 'XXXX unknown packet', map(lambda x: hex(ord(x)), buff)
        except socket.timeout, e : print e
        except socket.error, e : print e

        return pkt

    def handle_bcast_data(self, pkt):

        t_now = datetime.datetime.now()
        try :
            self.elapsed += t_now - self.prev_ts
            self.bc_pkt_cnt += 1
            self.prev_ts = t_now
        except AttributeError : 
            self.prev_ts = t_now
            self.elapsed = datetime.timedelta(seconds=0)
            self.bc_pkt_cnt = 0
           
            
        bcast_data = board_data_type.data_factory(self.policy, self.policy_map)
        bcast_data.decode(pkt.to_buffer())

        for name, handler in self.bcast_data_handler.iteritems() :
            handler(bcast_data)
            
        return bcast_data

    def test_packets(self):

        # brute force
        for cmd_name in self.packets :
            if cmd_name.startswith('GET_') \
               and cmd_name.find('BROADCAST_PACKET') == -1 \
               and cmd_name.find('GET_ACTIVE_BOARDS') == -1 :
                if self.send_tcp_cmd(cmd_name):
                    pkt = self.recv_tcp_reply()
                    if 0 and pkt :
                        #print pkt.toDict()
                        print pkt.name, pkt.get_data()

        print "[**] %s test packets passed" % self.__class__
        print "[***] ID %d adress %s" % (self.bId, self.bAddr)

# ##############################################################################
#
# ##############################################################################

class FtBoard(Board):

    policy_map = policy_maps.ft_policy_map

    board_type = 0x03

    def __init__(self, bId, bType, bAddr, controller, bPort=23) :

        Board.__init__(self, bId, bType, bAddr, bPort, controller)
        self.packets = FT_packets

        #self.bcast_data_handler['zmq_pub'] = test_lib.zmq_pub_FT_bcastData

        # get some info from board
        self.send_tcp_cmd('GET_BOARD_TYPE')
        self.recv_tcp_reply()
        self.send_tcp_cmd('GET_FIRMWARE_VERSION')
        self.recv_tcp_reply()

        self.send_tcp_cmd('CALIBRATE_OFFSETS')
        time.sleep(1)

        self.test_packets()

    def handle_bcast_data(self, pkt):

        Board.handle_bcast_data(self, pkt)

        """
        cbuff  = (ctypes.c_char * 144 * 6)()
        cbuff[pkt.get_data()[0]-1] = ctypes.create_string_buffer(pkt.to_buffer()[4:-1],144)
        # this handler use a calibration matrix ....
        test_lib.zmq_pub_FT_bcastData(zpub_sock, pkt.get_data()[0], cbuff)
        """
        
# ##############################################################################
#
# ##############################################################################

class McBoard(Board):

    policy_map = policy_maps.mc_policy_map

    board_type = 0x02

    def __init__(self, bId, bType, bAddr, controller, bPort=23) :

        Board.__init__(self, bId, bType, bAddr, bPort, controller)
        self.packets = MC_packets
        
        #
        self.test_packets()
        # get some info from board
        '''
        self.send_tcp_cmd('GET_BOARD_TYPE')
        pkt = self.recv_tcp_reply()
        print struct.unpack("=B", pkt._data)[0]
        self.send_tcp_cmd('GET_FIRMWARE_VERSION')
        pkt = self.recv_tcp_reply()
        print struct.unpack("=BB", pkt._data)[0]
        '''
        self.send_tcp_cmd('GET_MIN_POSITION')
        pkt = self.recv_tcp_reply()
        print struct.unpack("=i", pkt._data)[0]
        self.send_tcp_cmd('GET_MAX_POSITION')
        pkt = self.recv_tcp_reply()
        print struct.unpack("=i", pkt._data)[0]
        self.send_tcp_cmd('GET_MAX_VELOCITY')
        pkt = self.recv_tcp_reply()
        print struct.unpack("=H", pkt._data)[0]
        '''
        self.send_tcp_cmd('SET_MAX_VELOCITY',
                          [ord(x) for x in struct.pack("=H", 4000)])
        self.send_tcp_cmd('GET_MAX_VELOCITY')
        pkt = self.recv_tcp_reply()
        print struct.unpack("=H", pkt._data)
        '''
        self.gS = 1
        self.send_tcp_cmd('GET_PID_GAINS',[self.gS])
        pkt = self.recv_tcp_reply()
        self.gS,self.P,self.I,self.D = struct.unpack("=Biii", pkt._data)
        print 'PID Gains', self.gS,self.P,self.I,self.D
        self.send_tcp_cmd('GET_PID_GAIN_SCALE',[self.gS])
        pkt = self.recv_tcp_reply()
        print 'PID Gains scale', struct.unpack("=Biii", pkt._data)
        
        '''
        if self.send_tcp_cmd('SET_PID_GAINS',
                             [ord(x) for x in
                              struct.pack("=Biii",self.gS,self.P+5000,0,0)]) :
            time.sleep(1)
            #
            while 1 :
                self.send_tcp_cmd('GET_PID_GAINS',[self.gS])
                pkt = self.recv_tcp_reply()
                if pkt :
                    self.gS,self.P,self.I,self.D = struct.unpack("=Biii", pkt._data)
                    print 'PID Gains', self.gS,self.P,self.I,self.D
                    break
                else :
                    time.sleep(1)
       '''         
        
    def ctrl_pos(self, bcast_data):
        
        try :
            try : self.prev_pos
            except : self.prev_pos = 0
            
            #print abs(bcast_data.Position - self.prev_pos)
            #print '>>',bcast_data.Position - bcast_data.Target_Pos
                
            freq_Hz = 0.5
            A = 100000
            # sine    y(t) = A sin (2 PI freq t + teta) 
            target_pos = A * math.sin(2.0 * math.pi * freq_Hz * time.time())
            # square  y(t) = sgn ( A sin (2 PI freq t + teta) )
            target_pos = A if math.sin(2.0 * math.pi * freq_Hz * time.time()) > 0 else -A
            # tringle y(t) = A/PI_2 asin(sin(2 PI freq t + teta))
            #target_pos = A / (math.pi/2) * math.asin(math.sin(2.0 * math.pi * freq_Hz * time.time()))
            # saw     y(t) = A 2 (t/a - floor(t/a + 1/2)
            #target_pos = A * 2 * (time.time()*freq_Hz - math.floor(time.time()*freq_Hz + 0.5))

            self.controller.tx_upd('SET_DESIRED_POSITION',
                                   cmd_data=[ord(x) for x in struct.pack('=i',target_pos)],
                                   udp_port=[23])
            
            self.prev_pos = bcast_data.Position
            
        except Exception, e :
            print e
            print traceback.print_exc(file=sys.stdout)
        
        

# ##############################################################################
#
# ##############################################################################

def zmq_data_forwarder(pub_proxy_port=ZMQ_PUB_PROXY_PORT,
                       sub_proxy_port=ZMQ_SUB_PROXY_PORT):
    ''' forwarder for the publish/subscribe messaging pattern
        pubsub proxy
    '''
    _context = zmq.Context()
    frontend = _context.socket(zmq.PUB)
    frontend.bind("tcp://*:%d"%pub_proxy_port)
    backend = _context.socket(zmq.SUB)
    backend.setsockopt(zmq.SUBSCRIBE, "")
    #backend.setsockopt(zmq.HWM, 1)
    backend.bind("tcp://*:%d"%sub_proxy_port)
    print "Setup Proxy Sub:%d --> Pub:%d" % (sub_proxy_port,pub_proxy_port)
    zmq.device(zmq.FORWARDER, frontend, backend)
    print 'exit ... Proxy'


# ##############################################################################
#
# ##############################################################################
                
class Boards_controller(object):

    def __init__(self, host_ip, conf):

        self.boards_conf = conf
        self.active_boards = {}
        self.tcp_socks = {}

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_sock.settimeout(1)
        self.udp_sock.bind(host_ip)
     
        self.rx_udp_th = threading.Thread(target=self.rx_udp_thread, args=())
        self.rx_udp_th.start() 

        if use_zmq :
            self.trajectory_th = threading.Thread(target=self.traj_gen_thread, args=())
            self.trajectory_th.start()
            self.zrep_th = threading.Thread(target=self.zmq_rep_thread, args=())
            self.zrep_th.start() 
        
        self.tx_upd('GET_ACTIVE_BOARDS', udp_port=[23,24])

        
    def start_stop_bc(self, start=False):
        
        if start :
            # start bcast data
            for b in self.active_boards.itervalues():
                # !!!!!
                if b.bId <= 15 :
                    b.start()
        else :
            for b in self.active_boards.itervalues():
                b.send_tcp_cmd('SET_BCAST_RATE', cmd_data=[0x0,0x0])

    def start_stop_ctrl_boards(self, start=False):
        
        start_stop = 0x3 if start else 0x1
        # start board controller
        # !!!!
        self.tx_upd('POSITION_MOVE',
                    cmd_data=[ord(x) for x in struct.pack('=15B',*([start_stop]*15))],
                    udp_port=[23])

        
    def tx_upd(self, cmd_name, cmd_data=[], boards=[], udp_port=[]):

        try : 
            pkt = all_packets[cmd_name]
            if len(cmd_data):
                pkt = pkt.from_data(cmd_data)
        except KeyError : 
            print 'FAIL tx', pkt.name
            return False
        for port in udp_port :
            self.udp_sock.sendto(pkt.to_buffer(),('255.255.255.255',port))
        return True

    
    def rx_udp_thread(self):
        ''' '''
        try :
            while not loop.is_set() :
                try : buff = self.udp_sock.recv(1024)
                except socket.timeout : continue
                except socket.error, e: 
                    print e
                    break
                pkt = on_rx_pkt(buff)

                if pkt is not None :
                    if pkt.name == 'REPLY_ACTIVE_BOARDS' :
                        #
                        board = board_factory(pkt.get_data(), self)
                        self.active_boards[board.bId] = board
                        
                    if pkt.name.find('BCAST_DATA_PACKET') > -1 :
                        bId = pkt.get_data()[0]
                        try : self.active_boards[bId]
                        except IndexError :
                            print '[EE] Rx packet from unknown board'
                            continue
                        try : 
                            bcast_data = self.active_boards[bId].handle_bcast_data(pkt)
                        except IndexError :
                            print '[EE] Rx wrong BC packet %d' % bId

        except Exception, e:
            print e
            traceback.print_exc()
        print 'rx_udp_thread .. exit'


    def traj_gen_thread(self):
        ''' receive udp command such as
            - SET_DESIRED_POSITION
            - SET_DESIRED_VELOCITY
            - SET_DESIRED_TORQUE
        '''
        zpull_sock = zcontext.socket(zmq.PULL)
        zpull_sock.bind("tcp://*:%d"%ZMQ_PULL_PORT)
        
        while not loop.is_set():
            
            try : 
                ts,cmd,targets = zpull_sock.recv_multipart()
                ts, = struct.unpack('d', ts)
                cmd, = struct.unpack('%ds'%len(cmd), cmd)
                #print time.time() - ts, ts
                self.tx_upd(cmd,
                            cmd_data=[ord(x) for x in targets],
                            udp_port=[23])
            except zmq.ZMQError, e :
                print e
        
        zpull_sock.close()
        print 'traj_gen_thread .. exit'

        
    def zmq_rep_thread(self):
        
        zrep_sock = zcontext.socket(zmq.REP)
        zrep_sock.setsockopt(zmq.IDENTITY, 'zzz')
        zrep_sock.bind("tcp://*:%d"% ZMQ_REP_PORT)
        
        while not loop.is_set() :
            try : 
                proto, id, op, buff = zrep_sock.recv_multipart()
                print "Received request: ", proto, id, op
                if proto == 'tcp':
                    if op in ['get','set'] :
                        board = self.active_boards[int(id)]
                        board.tcp_sock.send(buff)
                        if op == 'get' :
                            buff = board.tcp_sock.recv(1024)
                            zrep_sock.send(buff)
                else :
                    zrep_sock.send("REP")
            except zmq.ZMQError, e :
                print e
        
        zrep_sock.close()
        print 'zmq_rep_thread .. exit'

        
    def close(self):

        self.start_stop_bc()
        self.start_stop_ctrl_boards()

        # stop threads activity
        loop.set()
        self.rx_udp_th.join(1)
        if use_zmq :
            self.trajectory_th.join(1)
            self.zrep_th.join(1)
        
        self.udp_sock.close()
        for b in self.active_boards.itervalues():
            b.tcp_sock.close()
            #            
            print b.bId, b.bc_pkt_cnt, b.elapsed / b.bc_pkt_cnt
        

# ##############################################################################
#
# ##############################################################################

if __name__ == '__main__' :

    from multiprocessing import Process
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option("-i", "--ip_addr", action="store", type="string", dest="ip_addr", default='169.254.89.9')
    parser.add_option("-p", "--port", action="store", type="int", dest="port", default=12345)
    parser.add_option("-z", "--zproxy", action="store_true", dest="use_zmq", default=False)
    (options, args) = parser.parse_args()

    use_zmq = options.use_zmq and has_zmq

    boards_conf = {
        # board_type MC
        2: {
            # board_ids
            1: {
                #'policy': 'Position|Torque|Velocity|Current|Temp_DC',
                'policy': 'Position|Torque|Velocity|Abs_pos|Rel_pos',
                #'policy': 'Position|Torque|Velocity|Target_Pos|PID_out|PID_err',
                #'policy': 'Position',
                'bcast_rate': 10, # /2 = ms
                },
            # default 
            -1: {
                'policy': 'Position|Torque|Velocity|Abs_pos|Rel_pos',
                #'policy': 'Position',
                'bcast_rate': 10, # /2 = ms
                },
        },
        # board_type FT
        3 : {
            # board_ids
            1: {
                'policy': 'Torque',
                'bcast_rate': 10,
            },
                        # default 
            -1: {
                'policy': 'Torque',
                'bcast_rate': 10, # /2 = ms
                },

        }
    }

    if use_zmq :
        zproxy = Process(target=zmq_data_forwarder, args=())
        zproxy.start()
        #zbroker = Process(target=zmq_req_rep_queue, args=())
        #zbroker.start()

    try :
        board_ctlr = Boards_controller((options.ip_addr,options.port), boards_conf)
        while True:
            try :
                print "Enter ... ^C to stop"
                cmd = sys.__stdin__.readline()[:-1]
                # match s or S
                if cmd.lower() == 's' :
                    # s : start -- S : stop
                    board_ctlr.start_stop_bc(cmd=='s')
                elif cmd.lower() == 'a' :
                    # a : start -- A : stop
                    board_ctlr.start_stop_ctrl_boards(cmd=='a')
                else :
                    break
            except KeyboardInterrupt, e :
                print e
                break # exit while True
        board_ctlr.close()
    except Exception, e :
        print e

    print 'clean up ....'            
    if has_zmq :
        zpub_sock.close()
        zcontext.term()
        if use_zmq :
            zproxy.terminate()
            #zbroker.terminate()

    print 'exit !!'
