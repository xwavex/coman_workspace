/*
   Boards_exception.h

   Copyright (C) 2013 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifndef __BOARDS_EXCEPTION_H__
#define __BOARDS_EXCEPTION_H__

#include <stdexcept>

class boards_error : public std::runtime_error {
public:
    boards_error (const std::string& what_arg):
        std::runtime_error(what_arg) {}
};

class boards_warn : public std::runtime_error {
public:
    boards_warn (const std::string& what_arg):
        std::runtime_error(what_arg) {}
};


#endif

