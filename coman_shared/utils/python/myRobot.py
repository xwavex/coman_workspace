from collections import defaultdict
import operator
import math
import time
import pprint

def homing_pos():
    # degree
    home_pos = [0,0,0,0,0,0,0,-2,0,0,0,0,-2,0,0] 
    return map(lambda x: operator.mul(math.radians(x),1e5), home_pos)

def homing_vel():
    #degree/s
    #home_vel = [8,8,8,8,8,8,8,8,8,8,8,8,8,8,8] 
    home_vel = [2] * 15 
    return map(lambda x: operator.mul(math.radians(x),1e3), home_vel)


class Robot :
    
    def __init__(self):
        
        self.bc_data = defaultdict()
        self.curr_pos = [1e9] * 15
        self.start_moving = False
        self.bset = set()
        
    def __call__(self, *args):
        
        if self.sense(*args) :
            
            dpos,dvel = homing_pos(),homing_vel()
        
            print self.curr_pos
        
            epos = [math.degrees(abs(x-y)/1e5) > 0.5 for x,y in zip(self.curr_pos,dpos)]
            #print 'ee', epos
            
            if not sum(epos) :
                self.start_moving = True
                self.start_time = time.time()
            
            if self.start_moving :
                dpos = self.move()
            
            return dpos,dvel
        
        return [],[]

    def sense(self, *args):
        
        board,data = args
        self.bc_data[board] = data

        board_id = int(board[len('board_'):])
        self.bset.add(board_id)
        self.curr_pos[board_id-1] = self.bc_data[board]['Position']
        
        if self.bset == set(range(1,16)) :
            self.bset = set()
            return True
        return False
        
        
    
    def move(self):
        
        pos = [0] * 15
        freq_Hz = 0.5
        t = time.time() - self.start_time
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

    