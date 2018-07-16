#! /usr/bin/python
# -*- coding: utf-8 -*-

import struct
import operator
import StringIO
import pprint
import json
import string

def checksum(data):
    return reduce(operator.sub,[0]+data) % 256

def check_byte(_byte):
    ''' return an int '''
    if type(_byte) not in [type(''),type(u'')] :
        _byte = '0x'+str(_byte)
    byte = string.atoi(_byte,16)
    assert(byte <= 0xFF)
    return byte


class dec_enc(struct.Struct):

    def __init__(self):
        
        self.__dict__.update(dict.fromkeys(self._fields_))
        fmt = self._byte_order_ + self._format_
        struct.Struct.__init__(self, fmt)
        self._buf = StringIO.StringIO()
        
    def encode(self):

        self._buf.seek(0)  #= StringIO.StringIO()
        self._buf.write(self.pack(*[getattr(self, k) for k in self._fields_]))  
        return self._buf.getvalue()

    def decode(self, data, offset=0):

        _data = data[offset:]
        buf = _data if hasattr(_data, 'read') else StringIO.StringIO(_data)
        for k,v in zip(self._fields_, self.unpack(buf.read(self.size))) :
            setattr(self, k, v)
        
    def toDict(self, all_fields=True):
        
        return dict((k,v)
                    for k,v in self.__dict__.iteritems()
                    if k in self._fields_ and (not k.startswith('_') or all_fields) )

    def __str__(self):
        
        return str(self.toDict())

# ##############################################################################
#        
# ##############################################################################

class board_pkt(dec_enc):

    def __init__(self, name,
                 extra_fields=[], extra_fields_fmt='',
                 byte_order='='):
        
        self.name = name
        self.extra_fields = extra_fields
        self.extra_fields_fmt = extra_fields_fmt
        self._fields_ = tuple(['_header','_n_bytes','_cmd'] + self.extra_fields + ['_chk'])
        self._format_ = 'BBB' + self.extra_fields_fmt + 'B'
        self._byte_order_ = byte_order
        dec_enc.__init__(self)
        
    @classmethod
    def from_excel(cls, row_cells):
        ''' '''
        data = []
        name = row_cells[0].split(' ')[0]
        nbytes = row_cells[2]
        # n_bytes and CHKSUM are related : nbytes != 0 <==> data_cmd 
        # ... but n_bytes cell could BE 4xn so use CHECKSUM cell  
        has_chksum_cell = row_cells[-1] == 'CHKSUM'
        if has_chksum_cell :
            nbytes = row_cells.index('CHKSUM') - 4
            row_cells[2] = nbytes
            data = [0] * nbytes
            new_pkt = cls(name,extra_fields=['_data'],extra_fields_fmt='%ds'%nbytes)
        else :
            new_pkt = cls(name)
            
        try : blob = map(check_byte, row_cells[1:4])
        except Exception, e : 
            #print row_cells[1:4]
            raise e
        
        new_pkt.decode("".join(map(chr,blob+data+[checksum(blob+data)])))
        return new_pkt
        
    def from_buffer(self, _buff):
        ''' get nbytes from buffer ...'''
        nbytes = ord(_buff[1])
        new_pkt = board_pkt(self.name,
                            extra_fields=self.extra_fields,
                            extra_fields_fmt='%ds'%nbytes)
        new_pkt.decode(_buff)
        return new_pkt

    def from_data(self, _data):
        ''' get nbytes from buffer ...'''
        nbytes = len(_data)
        new_pkt = board_pkt(self.name,
                            extra_fields=self.extra_fields,
                            extra_fields_fmt='%ds'%nbytes)
        new_pkt._header = self._header
        new_pkt._n_bytes = nbytes
        new_pkt._cmd = self._cmd
        new_pkt._chk = 0
        new_pkt.set_data(data=_data)
        
        
        return new_pkt


    def to_buffer(self, data=[]):
        
        self.set_data(data)
        return self.encode()
        
    def set_data(self, data):
        
        if len(data) and hasattr(self, '_data') :
            self._data = "".join([chr(x) for x in data])
            '''
            print '>>>',self._data
            print self._format_
            print self._fields_
            print self._header
            print self._n_bytes
            print self._cmd
            print len(self._data)
            print self.encode()
            '''
        self._chk = checksum([ord(x) for x in self.encode()[:-1]])
        
    def get_data(self):
        
        if hasattr(self, '_data') :
            return [ord(x) for x in self._data]
        return []

# ##############################################################################
#        
# ##############################################################################

class bcast_data_t(dec_enc):
 
    def __init__(self, fields, fmt, policy_number=None ,byte_order='='):
        
        self._fields_ = tuple(['_header','_n_bytes','_command', '_board_id'] + fields + ['_chk'])
        self._format_ = 'BBBB' + fmt + 'B'
        self._byte_order_ = byte_order
        self._policy_number = policy_number
        dec_enc.__init__(self)


def data_factory(policy, policy_map):
    ''' p0|p1|...pN '''
    
    #subset = dict((k, policy_map[k]) for k in policy.split('|'))
    # vector of tuple
    pol = []
    for token in policy.split('|'):
        try : p = policy_map[token]
        except KeyError : continue
        if p.has_key('fields'):
            ffields = p['fields']
        else :
            ffields = []
        pol.append((token,p['pos'],p['nb'],p['fmt'],ffields))
    
    # sort with key 'pos'
    pol = sorted(pol, key=operator.itemgetter(1))
    fields = [] 
    fmt = ''
    bits = []
    for _name,_pos,_nbytes,_fmt,_subfields in pol :
        if len(_subfields):
            for f in _subfields :
                fields.append(f)
        else :
            fields.append(_name)
        fmt += _fmt
        bits.append(_pos)

    bits.sort(reverse=True)
    policy_number = reduce(lambda x,y: x+y ,map(lambda x: 2**x, bits))
        
    return bcast_data_t(fields, fmt, policy_number=policy_number)

    
        