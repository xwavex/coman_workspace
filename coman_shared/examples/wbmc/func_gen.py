#! /bin/env python

import sys

print 'gen .h'

with open('wbmc_func.h','w') as f :
    sys.stdout = f
    print '#include <Eigen/Dense>'
    print
    print 'typedef float (*func_prt)(Eigen::Matrix<float,29,1>&, Eigen::Matrix<float,29,1>&);'
    print 'extern func_prt tau_func_ptr[29];'    
    print 'extern func_prt inertia_func_ptr[23][23];'
    print
    for i in range(1,30) :
        print
        print '''float tau_%d(Eigen::Matrix<float,29,1>& , Eigen::Matrix<float,29,1>& ); ''' % (i)
    print
    
    for i in range(1,24) :
        for j in range(1,24) :
            print
            print '''float inertia_%d_%d(Eigen::Matrix<float,29,1>& , Eigen::Matrix<float,29,1>& ); ''' % (i,j)
        
sys.stdout = sys.__stdout__
print 'gen .c'

with open('wbmc_func.cpp','w') as f :
    sys.stdout = f
    print '#include "wbmc_func.h"'
    print 'extern float spin(void);'
    for i in range(1,30) :
        print
        print '''float tau_%d(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}''' % (i)
    print
    print 'func_prt tau_func_ptr[29] = {'
    for i in range(1,30) :
        print 'tau_%d,' % i,
    print '};'
    print
    
    for i in range(1,24) :
        for j in range(1,24) :
            print
            print '''float inertia_%d_%d(Eigen::Matrix<float,29,1>& s, Eigen::Matrix<float,29,1>& c) {
        return spin();
}''' % (i,j)
    print
    print 'func_prt inertia_func_ptr[23][23] = {'
    for i in range(1,24) :
        for j in range(1,24) :
            print 'inertia_%d_%d,' % (i,j),
    print '};'

        