#! /usr/bin/env python

import collections
import itertools
import string
import operator

from openpyxl.reader.excel import load_workbook
import board_data_type


def get_board_packets(iface_xlfile, sheet='Sheet1', verbose=False) :

    wb = load_workbook(filename=iface_xlfile)
    ws_iface = wb.get_sheet_by_name(name = sheet)

    packets = {}
    counter = itertools.count(1)
    
    for i,r in itertools.izip(counter,ws_iface.rows) :
        try :
            if r[0].value != None and string.atoi(r[1].value,16) >= 0xFD : # and r[0].value[:4] in ['GET_', 'SET_'] : 
                # row is <Cell Interface> tuple
                cols_with_values = [c.value for c in r if c.value != None]
                pkt = board_data_type.board_pkt.from_excel(cols_with_values)
                packets[pkt.name] = pkt 
        except Exception, e :
            if verbose :
                print 'Skip', i ,'>>', r[0].value, '<<'
                print e
                
    
    return packets
        

if __name__ == '__main__' :
    
    import pprint
    import sys
    
    for f,sheet in [
        (r'Motor Controller Interface.xlsx','Interface'),
        #(r'DC Motor Controller Interface.xlsx','Interface'),
        #(r'TorqueDSP Interface.xlsx', 'Sheet1'),
        ] :
        
        print 'press enter to process %s' % f
        sys.__stdin__.readline()
        packets = get_board_packets(f, sheet=sheet, verbose=True)
        pprint.pprint(packets)
        print
    