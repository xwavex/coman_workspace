#! /usr/bin/env python

import os
import sys
import pprint
import itertools
import numpy
#from plotting import *
from matplotlib import pyplot, mlab
from matplotlib.backends.backend_pdf import PdfPages

BOARDS_NUM = 27

rt_prefix = 'rt_log'
xeno_prefix = 'xeno_log'
#dir_prefix = xeno_prefix

file_prefix = 'log_bId_%d.txt'

def stat(v) :

    return {
        'len': len(v), 
        'm': numpy.amin(v),
        'M':  numpy.amax(v),
        'mean': numpy.mean(v),
        'std': numpy.std(v),
        'var': numpy.var(v),
        }

def do_plot(filename, plot_idx):
    
    v = [(int(l.split('\t')[0])/1000) for l in file(filename).read().splitlines()]
    dt = [(j-i) for i, j in itertools.izip(v[:-1], v[1:])] 
    #dt = 1000 + 35*numpy.random.randn(10000)
    
    sts = stat(dt)
    print  filename
    pprint.pprint(sts)
    print
    
    ax = pyplot.subplot(111)
    #ax = pyplot.subplot(120+(plot_idx%2)+1)
    #ax = pyplot.subplot(220+(plot_idx%4)+1)
    
    n, bins, patches = pyplot.hist(dt, bins=500, label=os.path.dirname(filename))
    #y = mlab.normpdf( bins, sts['mean'], sts['std'])
    #l = pyplot.plot(bins, y, 'r--', linewidth=3)
    #pyplot.text(-10,-10, pprint.pformat(sts))
    pyplot.annotate(pprint.pformat(sts), xy=(0.005, 0.7), xycoords='axes fraction')
    pyplot.xlabel('freq (microsec)')
    pyplot.ylabel('samples')
    pyplot.legend()
    pyplot.grid(True)

def filename_gen():
    
    for idx in range(BOARDS_NUM) :
        filename = file_prefix % (idx+1)
        yield filename
    filename = 'master_log.txt'
    yield filename
    
    
if __name__ == '__main__' :    

    pdf = PdfPages('latency.pdf')

    k=0
    for filename in filename_gen()  :
        for dir_prefix in [rt_prefix, xeno_prefix] :
        
            filepath = os.path.join(dir_prefix,filename)
            if os.path.isfile(filepath) :
                k +=1 
                do_plot(filepath, k)
                pyplot.title(filename)
            else :
                print 'NOT FOUND', filepath
                
        try :
            # if file is not found NO figure is avalaible
            # and savefig raise ValueError
            pdf.savefig()
            pyplot.close()
        except ValueError : pass
        
   
    pdf.close()
    
