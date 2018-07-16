#! /usr/bin/env python

import sys
import time
import datetime
import threading
import zmq
import json
import struct
import math
import itertools
import operator

import read_xl
import myRobot

context = zmq.Context()

ZMQ_REP_PORT = 6969
ZMQ_PULL_PORT = 7777

BOARD_IFACE_HOST = 'tcp://localhost'
#BOARD_IFACE_HOST = 'tcp://10.255.32.77'
DEFAULT_ZMQ_PUB = BOARD_IFACE_HOST+':5555'
DEFAULT_ZMQ_PULLER = '%s:%d' % (BOARD_IFACE_HOST,ZMQ_PULL_PORT)
DEFAULT_ZMQ_REP = '%s:%d' % (BOARD_IFACE_HOST,ZMQ_REP_PORT)

freq_Hz = 0.2
Amplitude = 100000
start_time = time.time()

# lower body motor
MAX_MOTOR_BOARDS = 15

def trj_sin(A, freq_Hz, teta): 
    # sine    y(t) = A sin (2 PI freq t + teta) 
    return A * math.sin(2.0 * math.pi * freq_Hz * time.time() + teta)

def trj_square(A, freq_Hz, teta): 
    # square  y(t) = sgn ( A sin (2 PI freq t + teta) )
    return A if trj_sin(1, freq_Hz, teta) > 0 else -A

def trj_triangle(A, freq_Hz, teta):
    # triangle y(t) = A/PI_2 asin(sin(2 PI freq t + teta))
    return A / (math.pi/2) * math.asin(math.sin(2.0 * math.pi * freq_Hz * time.time()))

def trj_saw(A, freq_Hz, teta):
    # saw     y(t) = A 2 (t/a - floor(t/a + 1/2)
    return A * 2 * (time.time()*freq_Hz - math.floor(time.time()*freq_Hz + 0.5))

def homing_pos():
    # degree
    home_pos = [0,0,0,0,0,0,0,-1,0,0,0,0,-1,0,0] 
    return map(lambda x: operator.mul(math.radians(x),1e5), home_pos)

def homing_vel():
    #degree/s
    #home_vel = [8,8,8,8,8,8,8,8,8,8,8,8,8,8,8] 
    home_vel = [2] * 15 
    return map(lambda x: operator.mul(math.radians(x),1e3), home_vel)

"""
traj[0]=0;		// wait yaw, so far non exist
traj[1]=DEGTORAD(2)*(1-cos(2*pi/input_T*dtime*dT));		// wait pitch
traj[2]=DEGTORAD(0)*sin(2*pi/input_T*dtime*dT);			// wait roll
traj[3]=DEGTORAD(10)*(1-cos(2*pi/input_T*dtime*dT));	// right hip pitch
traj[4]=DEGTORAD(10)*(1-cos(2*pi/input_T*dtime*dT));	// left hip pitch 
traj[5]=DEGTORAD(-0)*(1-cos(2*pi/input_T*dtime*dT));	// right hip roll
traj[6]=DEGTORAD(-0)*(1-cos(2*pi/input_T*dtime*dT));	//right hip yaw
traj[7]=DEGTORAD(homePos[7])+DEGTORAD(-20)*(1-cos(2*pi/input_T*dtime*dT));	//right knee
traj[8]=DEGTORAD(10)*(1-cos(2*pi/input_T*dtime*dT));	//right ankle pitch
traj[9]=DEGTORAD(0)*(1-cos(2*pi/input_T*dtime*dT));		//right ankle roll			
traj[10]=DEGTORAD(-0)*(1-cos(2*pi/input_T*dtime*dT));	// left hip roll
traj[11]=DEGTORAD(-0)*(1-cos(2*pi/input_T*dtime*dT));	// left hip yaw
traj[12]=DEGTORAD(homePos[12])+DEGTORAD(-20)*(1-cos(2*pi/input_T*dtime*dT));	// left knee
traj[13]=DEGTORAD(10)*(1-cos(2*pi/input_T*dtime*dT));	//left ankle pitch
traj[14]=DEGTORAD(0)*(1-cos(2*pi/input_T*dtime*dT));	// left ankle roll
"""

def move():

    pos = [0] * 15
    t = time.time() - start_time
    pos [0] = 0
    pos [1] = math.radians(2) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [2] = math.radians(0) * math.sin(2.0 * math.pi * freq_Hz * t) * 1e5
    pos [3] = math.radians(10) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [4] = math.radians(10) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [5] = math.radians(0) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [6] = math.radians(0) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [7] = math.radians(-10) + math.radians(-20) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [8] = math.radians(10) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [9] = math.radians(0) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [10] = math.radians(0) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [11] = math.radians(0) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [12] = math.radians(-10) + math.radians(-20) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [13] = math.radians(10) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    pos [14] = math.radians(0) * (1-math.cos(2.0 * math.pi * freq_Hz * t)) * 1e5
    
    return pos


class TRJ_generator(threading.Thread):

    def __init__(self, *args, **kwargs):

        threading.Thread.__init__(self)
        self.ev = threading.Event()
        self.ev.clear()
        
        self.zmq_context = kwargs.get('zmq_context', zmq.Context())
        self.zmq_pub = kwargs.get('zmq_pub', DEFAULT_ZMQ_PUB)
        self.zmq_msg_sub = kwargs.get('zmq_msg_sub', '')
        self.signals = kwargs.get('signals', '')

        self.zpush = context.socket(zmq.PUSH)
        self.zpush.setsockopt(zmq.LINGER, 1)
        self.zpush.setsockopt(zmq.HWM, 1)
        self.zpush.connect(DEFAULT_ZMQ_PULLER)
        
        self.zreq = context.socket(zmq.REQ)
        self.zreq.connect(DEFAULT_ZMQ_REP)
        
        # Connect subscriber to publisher
        self.subscriber = self.zmq_context.socket(zmq.SUB)
        for msg in self.zmq_msg_sub.split(',') :
            self.subscriber.setsockopt(zmq.SUBSCRIBE, msg)
        self.subscriber.connect(self.zmq_pub)

        self.callback = kwargs.get('cb', self.trj) 
        #self.trj_fun = trj_sin
        
        self.robot = myRobot.Robot()

        
    def trj(self, *args):
        
        #for i in range(MAX_MOTOR_BOARDS) :
        #    targets_pos[i] = self.trj_fun(Amplitude,freq_Hz,0)
        
        targets_pos,targets_vel = self.robot(*args)
        if not len(targets_pos) :
            return
        
        ts = time.time()
        #print ts
        cmd = 'SET_DESIRED_VELOCITY'
        self.zpush.send_multipart([struct.pack('=d', ts),
                                   struct.pack('=%ds'%len(cmd), cmd),
                                   struct.pack('=%dh'%MAX_MOTOR_BOARDS, *targets_vel)])

        ts = time.time()
        #print ts
        cmd = 'SET_DESIRED_POSITION'
        self.zpush.send_multipart([struct.pack('=d', ts),
                                   struct.pack('=%ds'%len(cmd), cmd),
                                   struct.pack('=%di'%MAX_MOTOR_BOARDS, *targets_pos)])
        

        
    def run(self):
    
        prev = datetime.datetime.now()
        
        while not self.ev.is_set():
    
            if False :
                # open loop
                time.sleep(0.002)
                try : id,data = self.subscriber.recv_multipart(zmq.NOBLOCK)
                except ValueError : pass 
                except zmq.ZMQError : continue
            else :
                # blocked on receive bcast data
                try : id,data = self.subscriber.recv_multipart()
                except ValueError : pass 
            
            # send trajectory data
            self.callback(id, json.loads(data)) 
            
            now = datetime.datetime.now()
            elap = now - prev
            prev = now
            print elap
            #print id , data
            
        self.subscriber.close()
        self.zpush.close()
        self.zmq_context.term()
    

    
if __name__ == '__main__' :
    
    import sys
    from optparse import OptionParser
    
    parser = OptionParser()
    parser.add_option("--zmq-pub", action="store", type="string", dest="zmq_pub", default=DEFAULT_ZMQ_PUB)
    parser.add_option("--zmq-msg-sub", action="store", type="string", dest="zmq_msg_sub", default="")
    parser.add_option("--signals", action="store", type="string", dest="signals", default="")
    (options, args) = parser.parse_args()
    dict_opt = vars(options)

    
    MC_packets = read_xl.get_board_packets(r'Motor Controller Interface.xlsx', sheet='Interface')

    packets = dict(MC_packets.items() )
    packets_by_cmd = dict(zip([o._cmd for o in packets.values()], packets.values()))

    trj_map = {'sin': trj_sin,
               'saw': trj_saw,
               'tri': trj_triangle,
               'sqr': trj_square,
               }

    trj = TRJ_generator(dict_opt)

    # get/set some data from boards ?!?
    pkt = packets['GET_MIN_POSITION']
    trj.zreq.send_multipart(['tcp','2','get',pkt.to_buffer()])
    print struct.unpack("=BBBiB", trj.zreq.recv())
    pkt = packets['GET_MAX_POSITION']
    trj.zreq.send_multipart(['tcp','2','get',pkt.to_buffer()])
    print struct.unpack("=BBBiB", trj.zreq.recv())
    
    # start thread activity
    trj.start()
    try : 
        while True :
            print "Enter trj fun ...", trj_map.keys(), "or a A f F"
            cmd = sys.__stdin__.readline()[:-1]
            try : 
                trj.trj_fun = trj_map[cmd]
            except KeyError :
                for c in [c for c in itertools.islice(cmd,0,len(cmd)) if c.lower() in 'af'] :
                    if c == 'A' :
                        Amplitude += 1000
                    elif c == 'a':
                        Amplitude -= 1000
                    elif c == 'F':
                        freq_Hz += 0.1
                    elif c == 'f':
                        freq_Hz -= 0.1
                    print Amplitude,freq_Hz
                
    except KeyboardInterrupt :
        pass

    trj.ev.set()
    
    print 'exit'
