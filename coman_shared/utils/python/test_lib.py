#! /usr/bin/env python

import sys
import os
import time
import socket
import ctypes
import struct
import threading
import itertools
import operator
import zmq
import json
import numpy as np

from board_data_ctypes import getdict, FT, FT_mod 

ZMQ_PUB_PORT = 5555

CALIB_M = [
    (-0.008186973059527,  0.004878439074664, -0.000019563409693, -0.000001931087329, -0.000001963882431,  0.000007364790548),
    ( 0.000163576817971,  0.000212934882702, -0.001023916998652, -0.000013344898922,  0.000008021909262, -0.000000275924235),
    ( 0.000545268651639, -0.010048068974656,  0.000097696035479,  0.000004821219020,  0.000000882503694,  0.000019293554923),
    ( 0.000073777776651, -0.000421393994452, -0.001486481695203,  0.000000399156640, -0.000016749084807,  0.000000610170247),
    ( 0.007870472636240,  0.004918582859410, -0.000067410602863, -0.000002620259178,  0.000000602195113,  0.000020976610243),
    (-0.000184003280919, -0.000244568742653, -0.001177848977737,  0.000014938936824,  0.000009109908080,  0.000000501118998),
]



# ##############################################################################
#
# ##############################################################################

class LibError(BaseException):
    
    def __init__(self, *args):
        self.result = args[2]
        BaseException.__init__(self, args)

def myErrCheck(result, func, arguments):
    ''' all functions return 0 on success or a RoboLLIError '''
    if result :
        raise LibError(func,arguments,result)
    return result

def loadLibrary(obj, functions, dll_filename, path=None):
    ''' extend obj instance with dll functions '''
    if path :
        old_path = os.environ['PATH']
        os.environ['PATH'] = old_path + os.pathsep + os.path.abspath(path)
        #print os.environ['PATH']

    dll = ctypes.cdll.LoadLibrary(os.path.join(path,dll_filename))
    # dll exported functions
    for f in functions :
        # getattr could raise AttributeError
        fname = f[2:] if f.startswith('__') else f
        func_ptr = getattr(dll, fname)
        obj.__dict__[fname] = func_ptr 
        if not f.startswith('__') :
            obj.__dict__[fname].errcheck = myErrCheck
        
# ##############################################################################
#
# ##############################################################################

def convert_data(v):
  
    # force torque vector as np matrix
    ft_m = np.dot(np.matrix(CALIB_M), np.array(v))
    # force torque vector as list
    return np.squeeze(np.asarray(ft_m)).tolist()


def print_FT_bcastData(bID, buff):

    # per single board buffer
    # rfx rfy rfz rtx rty rtz X 6 cell 
    # 6 short int X 6 cell --> 72 bytes
    buff_char  = (ctypes.c_char * 12 * 6)()
    buff_short = (ctypes.c_short * 6 * 6)()

    print 'Board :', bID
    ctypes.memmove(buff_char,  buff[bID-1], ctypes.sizeof(buff_char))
    ctypes.memmove(buff_short, buff[bID-1], ctypes.sizeof(buff_short))
 
    for c,s in zip(buff_char[:1],buff_short[:1]) :
        for v in c : print hex(ord(v)),
        print
        for v in s : print v,
        print
        
                
def zmq_pub_FT_bcastData(zmq_sock, bID, buff):

    # per single board buffer
    # rfx rfy rfz rtx rty rtz X 6 cell 
    # 6 short int X 6 cell --> 72 bytes
    buff_short = (ctypes.c_short * 6 * 6)()
    ctypes.memmove(buff_short, buff[bID-1], ctypes.sizeof(buff_short))
    
    ft = FT()
    ft_mod = FT_mod()
            
    counter = itertools.count(1)
    for s in buff_short[:1] :
        pfix = '_%d_%d' % (bID,counter.next())
        
        ft_m = convert_data(s)
        ft_mod.ft_array.ftv = (ctypes.c_float * 6)(*ft_m)
        #zmq_sock.send_multipart(['ft_sens_mod'+pfix, json.dumps(getdict(ft_mod))])
        zmq_sock.send_multipart(['ft_xyz_mod' +pfix, json.dumps(getdict(ft_mod.ft_vect))])
        #print ft.ft_array_mod.ftv_mod[:6]
    
        ctypes.memmove(ctypes.addressof(ft), s, ctypes.sizeof(ft))
        #zmq_sock.send_multipart(['ft_sens'+pfix, json.dumps(getdict(ft))])
        zmq_sock.send_multipart(['ft_xyz' +pfix, json.dumps(getdict(ft.ft_vect))])
     

    
# ##############################################################################
#
# ##############################################################################

class BoardLib(object):
            
    # common lib exported functions
    board_functions = [
        'init_lib',
        'close_lib',
        'getBoardType',
        'getFWVersion',
        'getProgrammedFWVersion',
        'setBroadcastPolicy',
        'startBroadcastData',
        'stopBroadcastData',
        'getBroadcastData',
        'getBroadcastDataM',
        'getActiveBoards',
        # do not errcheck
        '__getErrorMsg',
    ]
    
    def __init__(self):
        
        self.boards_num = 0
        self.boards_ids = []

    def init(self, iface, port):
        
        # open tcp and udp socket and scan for devices
        self.init_lib(ctypes.create_string_buffer(iface), port)
        boards_ids = (ctypes.c_int * 10)()
        boards_num = ctypes.c_int()
        self.getActiveBoards(ctypes.byref(boards_num), boards_ids)
        self.boards_num = boards_num.value
        self.boards_ids = boards_ids[:self.boards_num]

        for bID in self.boards_ids :
            board_type = ctypes.c_byte()
            board_fw = (ctypes.c_byte * 2)()
            self.getBoardType(bID, ctypes.byref(board_type))
            print board_type.value
            self.getFWVersion(bID, board_fw)
            print board_fw[0], board_fw[1]
            
        try : self.private_init()
        except AttributeError : pass 

    def shutdown(self):
        
        for bID in self.boards_ids :
            self.stopBroadcastData(bID)
        self.close_lib()

            
# ##############################################################################
#
# ##############################################################################
            
class FtLib(BoardLib):

    # lib exported functions
    ft_functions = BoardLib.board_functions + [
        'calibrateOffsets',
        'setAverageSamples',
        'setMatrixRow',
    ]

    def __init__(self, lib_name, lib_path, zcontext):
        
        BoardLib.__init__(self)
        loadLibrary(self, FtLib.ft_functions, lib_name, lib_path)
        self.socket = zcontext.socket(zmq.PUB)
        zmq_sock = "tcp://*:%d"%ZMQ_PUB_PORT
        self.socket.bind(zmq_sock)
        print 'zmq pub bind on', zmq_sock

    def getBCdata(self, loop):

        # all board buffer --> MAX_BCAST_DATA_SIZE * MAX_SENSORS_NUMBER
        MAX_BCAST_DATA_SIZE = 144
        MAX_SENSORS_NUMBER = 6
        buff  = (ctypes.c_char * MAX_BCAST_DATA_SIZE * MAX_SENSORS_NUMBER)()
    
        # one sensor
        ft = FT()
        
        while not loop.is_set() :
            try :
                if 1 :
                    # in the depth it call select !?!?
                    self.getBroadcastData(buff)
                    for bID in self.boards_ids :
                        print_FT_bcastData(bID, buff)
                        zmq_pub_FT_bcastData(self.socket, bID, buff)
                    time.sleep(0)
                else :
                    
                    buff_short = (ctypes.c_short * 6 * 6)()
                    for bID in self.boards_ids :
                        # !!! one sensor ID
                        self.getBroadcastDataM(bID, buff_short, ctypes.sizeof(buff_short))
                        print 'Board :', bID
                        # X 6 sensors
                        counter = itertools.count(1)
                        for v in buff_short[:1] :
                            ctypes.memmove(ctypes.addressof(ft), v, ctypes.sizeof(v))
                            for elem in v : print hex(elem),
                            print
                            for elem in v : print elem,
                            print
                            pfix = '_%d_%d' % (bID,counter.next())
                            self.socket.send_multipart(['ft_array'+pfix,json.dumps(getdict(ft))])
                            self.socket.send_multipart(['ft_xyz' +pfix,json.dumps(getdict(ft.ft_vect))])
    
                    time.sleep(0)
    
    
            except LibError, e :
                self.getErrorMsg.restype = ctypes.c_char_p
                print '>>>', self.getErrorMsg()
                time.sleep(3)

# ##############################################################################
#
# ##############################################################################
            
class McLib(BoardLib):

    # all board buffer --> MAX_BCAST_DATA_SIZE * MAX_MOTOR_NUMBER
    MAX_BCAST_DATA_SIZE = 46
    MAX_MOTOR_NUMBER = 30
        
    # lib exported functions
    mc_functions =  BoardLib.board_functions + [
        'getPIDGains',
        'setDesiredPosition',
    ]

    def __init__(self, lib_name, lib_path, zcontext):
        
        BoardLib.__init__(self)
        loadLibrary(self, McLib.mc_functions, lib_name, lib_path)

    def private_init(self): 
        
        gains = (ctypes.c_int * 3)()
        gain_type = 0
        for bID in self.boards_ids :
            self.getPIDGains(bID, gain_type, ctypes.byref(gains))
            for g in gains: print g,
            print
            
        
    def getBCdata(self, loop):

        buff  = (ctypes.c_char * McLib.MAX_BCAST_DATA_SIZE * McLib.MAX_MOTOR_NUMBER)()
        print 'Start getBCdata thread'
        
        while not loop.is_set() :
            try :
                if 1 :
                    # in the depth it call select !?!?
                    self.getBroadcastData(buff)
                    for bID in self.boards_ids :
                        self.print_BcData(bID, buff)
                        #zmq_pub_bcastData(self.socket, bID, buff)
                    time.sleep(0)
                    pos = (ctypes.c_int * 1)(100)
                    self.setDesiredPosition(pos)
    
            except LibError, e :
                self.getErrorMsg.restype = ctypes.c_char_p
                print '>>>', self.getErrorMsg()
                time.sleep(3)

    def print_BcData(self, bID, buff):
        
        for i in range(McLib.MAX_MOTOR_NUMBER) :
            for v in buff[bID-1] : print hex(ord(v)),
            print
                
                
# ##############################################################################
#
# ##############################################################################
                
if __name__ == '__main__' :
    
    zmq_context = zmq.Context()
    loop = threading.Event()
    
    #ft_obj = FtLib('libcFtSensor.so', '../build', zmq_context)
    mc_obj = McLib('libcMController.so', '../build', zmq_context)
    
    '''
    ft_obj.init("eth1:3", 23)
    if not len(ft_obj.boards_ids) :
        print 'NO FT board found ...'

    # now we know boards num and their ids 
    for bID in ft_obj.boards_ids :
        ft_obj.setBroadcastPolicy(bID, 0x0001)
        ft_obj.calibrateOffsets(bID)
        time.sleep(1)
        for i in range(6) :
            _cv = list(itertools.imap(operator.mul,CALIB_M[i],itertools.repeat(1000)))
            matrix_row = (ctypes.c_float * 6)(*_cv)
            #matrix_row = (ctypes.c_float * 6)(*CALIB_M[i])
            row_idx = ctypes.c_byte(i)
            #ft_obj.setMatrixRow(bID, row_idx, matrix_row) 
        ft_obj.startBroadcastData(bID, 200)
    
    '''
    
    mc_obj.init("eth1:3",23)
    if not len(mc_obj.boards_ids) :
        print 'NO MC board found ...'
        #exit()
    
    for bID in mc_obj.boards_ids :
        mc_obj.setBroadcastPolicy(bID, 0x0001)
        mc_obj.startBroadcastData(bID, 200)

        
    # create and start thread to get BC data
    #th_ft = threading.Thread(target=ft_obj.getBCdata, args=(loop,))
    #th_ft.start()
    th_mc = threading.Thread(target=mc_obj.getBCdata, args=(loop,))
    th_mc.start()
    
    print 'type to exit'
    try : sys.stdin.readline()
    except KeyboardInterrupt : pass
    finally :
        loop.set()
        #th_ft.join(1)
        th_mc.join(1)
    
    #ft_obj.shutdown()
    mc_obj.shutdown()
    print 'exit'
    
    