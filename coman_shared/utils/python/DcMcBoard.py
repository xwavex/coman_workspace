from boards_iface import Board


# ##############################################################################
#
# ##############################################################################

class DCMcBoard(Board):

    policy_map = {
        'Position': {'pos':0, 'nb':4*3, 'fmt':'i'*3, 'fields':['pos0','pos1','pos2']},
        'Velocity': {'pos':1, 'nb':2*3, 'fmt':'h'*3, 'fields':['vel0','vel1','vel2']},
        'Torque':   {'pos':2, 'nb':2*3, 'fmt':'h'*3, 'fields':['tor0','tor1','tor2']},
        'PID_out':  {'pos':3, 'nb':2*3, 'fmt':'h'*3, 'fields':['PID_out0','PID_out1','PID_out2']},
        'PID_err':  {'pos':4, 'nb':4*3, 'fmt':'i'*3, 'fields':['PID_err0','PID_err1','PID_err2']},
        'Current':  {'pos':5, 'nb':4*3, 'fmt':'i'*3, 'fields':['cur0','cur1','cur2']},
        'Temp_DC':  {'pos':6, 'nb':4,   'fmt':'i'},
        'Timestamp':{'pos':7, 'nb':4,   'fmt':'i'},
        # ---------------------------------------
        'Faults':   {'pos':8, 'nb':2,  'fmt':'B'*2, 'fields':['Faults','_Faults']},
        #'Ain_A':    {'pos':9, 'nb':2,  'fmt':'H'},
        #'Ain_B':    {'pos':10,'nb':4,  'fmt':'i'},
        'Abs_pos':  {'pos':11,'nb':2,  'fmt':'h'},
        'Rel_pos':  {'pos':15,'nb':2,  'fmt':'h'},
    }

    board_type = 0x04

    def __init__(self, bId, bType, bAddr, controller, bPort=23) :

        Board.__init__(self, bId, bType, bAddr, bPort, contoller)
        self.packets = DC_packets

        # get some info from board
        self.send_tcp_cmd('GET_BOARD_TYPE')
        self.recv_tcp_reply()
        self.send_tcp_cmd('GET_FIRMWARE_VERSION')
        self.recv_tcp_reply()

        self.test_packets()

        # at least set policy and bcast freq
        self.start()

