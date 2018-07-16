#! /usr/bin/env python
import time
#import string, random
from board_data_type import data_factory, bcast_data_t
import policy_maps

struct2c_map = {'b': 'char',
                'h': 'short',
                'i': 'int',
                'l': 'long',
                'q': 'long long',
                'f': 'float',
                'd': 'double',
                }

def c_type(k):
    try :
        return struct2c_map[k.lower()]+'\t' if k.islower() else 'unsigned '+ struct2c_map[k.lower()] 
    except KeyError :
        return '_unknown_map_'


def gen_header(out) :
    
    out.write('''/*
    broadcast.h

	Copyright (C) 2012 Italian Institute of Technology

	Developer:
        Alessio Margan (%s , alessio.margan@iit.it)

    Generated:
        %s
*/

#ifndef __BROADCAST_DATA_H__
#define __BROADCAST_DATA_H__

#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include <map>
#include <string>
typedef std::map<std::string, int> bc_data_map_t;

''' % (time.strftime("%Y", time.gmtime()), 
       time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())))

def gen_header_end(out) :

    out.write('\n#endif\n')
    

def gen_c_struct(kls, c_struct_name):

    def gen_print():
        struct_fields = [f for f in kls._fields_ if not f.startswith('_')]
        fmt = "%d\\t"*len(struct_fields)
        return fmt[:-2]+"\\n", ",".join(struct_fields)
    
    policy_num = kls._policy_number & 0xFFFF
    extra_policy_num = (kls._policy_number >> 16)
    lines = []
    lines.append("/*  " )
    lines.append(" * WARNING struct is generated with c_gen.py script" )
    lines.append(" * - policy value 0x%04X %s" % (policy_num,bin(policy_num)) )
    lines.append(" * - extra policy value 0x%04X %s" % (extra_policy_num,bin(extra_policy_num)) )
    lines.append(" *  " )
    lines.append(" */ " )
    lines.append("typedef struct {")
    for field,fmt in zip(kls._fields_, kls._format_) :
        lines.append("\t%s %s;" % (c_type(fmt),field))

    lines.append("\tconst unsigned short get_policy(void) { return 0x%04X; }" % policy_num)
    lines.append("\tconst unsigned short get_extra_policy(void) { return 0x%04X; }" % extra_policy_num)
    
    if 'Faults_0' in kls._fields_ : 
        lines.append("\tconst unsigned char faults(void) { return Faults_0; }")
    else :
        lines.append("\tconst unsigned short faults(void) { return 0x0; }")

    lines.append("\tvoid fprint(FILE *fp) {")
    lines.append("\t\tfprintf(fp, \"%s\", %s);" % gen_print())
    lines.append("\t}")
    
    lines.append("\tvoid sprint(char *buff, size_t size) {")
    lines.append("\t\tsnprintf(buff, size, \"%s\", %s);" % gen_print())
    lines.append("\t}")

    lines.append("\tvoid to_map(bc_data_map_t &bc_map) {")
    for field in [f for f in kls._fields_ if not f.startswith('_')]:
        #lines.append('\t\tbc_map["%s"] = boost::any(%s);' % (field,field))
        lines.append('\t\tbc_map["%s"] = %s;' % (field,field))
    lines.append("\t}")

    lines.append("} __attribute__((__packed__)) %s;" % c_struct_name)
    return lines


def write(policy, policy_map, c_struct_name, out):
    
    bc_data = data_factory(policy, policy_map)
    for line in gen_c_struct(bc_data, c_struct_name) :
        out.write("%s\n" % line)

    

if __name__ == '__main__' :
    
    with file('broadcast_data.h','w') as out :
        gen_header(out)
        
        policy = 'Position|Velocity|Torque|PID_out|PID_err|Current|Faults|Link_pos|Target_pos|Link_vel'
        #policy = 'Position|Torque|Velocity|PID_out|PID_err|Current|TT_2|TT_3|Ain_0|Ain_1|Piezo_out'
        write(policy, policy_maps.mc_policy_map, 'mc_bc_data_t', out)
        
        #policy = 'Position|Velocity|Torque|PID_err|PID_out|Current|Tendon_tor|Faults|Height|Hip_pos|Target_pos|Lin_enc_pos|Lin_enc_raw|Delta_tor|Lin_enc_vel'
        #policy = 'Position|Velocity|PID_out|Tendon_tor|Target_pos|Delta_tor'
        #write(policy, policy_maps.bigLeg_policy_map, 'mc_bc_data_t', out)
        
        policy = 'Mod_Torque'
        write(policy, policy_maps.ft_policy_map, 'ft_bc_data_t', out)
        
        gen_header_end(out)
        
    '''
    policy_map = {
        'Pollo': {'pos':0, 'nb':1, 'fmt':'b'},
        'Gatto': {'pos':1, 'nb':1, 'fmt':'B'},
        'Pizza': {'pos':2, 'nb':4, 'fmt':'i'},
        'Vino':  {'pos':3, 'nb':4, 'fmt':'I'},
        'Pasta': {'pos':4, 'nb':2, 'fmt':'h'},
        'Dolce': {'pos':5, 'nb':2, 'fmt':'H'},
        'Birra': {'pos':6, 'nb':4, 'fmt':'f'},
    }
    
    keys = policy_map.keys()
    random.shuffle(keys)
    bc_data = data_factory(string.join(keys,'|'), policy_map)
    for l in gen_c_struct(bc_data, 'dummy_t') :
        print l
    
    '''
