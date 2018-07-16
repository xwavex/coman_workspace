import ctypes


def getdict(struct):
    result = {}
    for field, _ in struct._fields_:
        value = getattr(struct, field)
        if hasattr(value, "_length_") and hasattr(value, "_type_"):
            # Probably an array
            value = list(value)
        elif hasattr(value, "_fields_"):
            # Probably another struct
            value = getdict(value)
        result[field] = value
    return result

# ##############################################################################
# force torque 
# ##############################################################################

#class FTarray(ctypes.BigEndianStructure):
class FTarray(ctypes.Structure):
    _fields_ = [
        ("ftv", ctypes.c_short * 6)
    ]

#class FTvect(ctypes.BigEndianStructure):
class FTvect(ctypes.Structure):
    _fields_ = [
        ("fx", ctypes.c_short),
        ("fy", ctypes.c_short),
        ("fz", ctypes.c_short),
        ("tx", ctypes.c_short),
        ("ty", ctypes.c_short),
        ("tz", ctypes.c_short),
    ]

class FT(ctypes.Union):
    _fields_ = [
        ("ft_array", FTarray ),
        ("ft_vect",  FTvect),
    ]

    
class FTarray_mod(ctypes.Structure):
    _fields_ = [
        ("ftv", ctypes.c_float * 6)
    ]

class FTvect_mod(ctypes.Structure):
    _fields_ = [
        ("fx", ctypes.c_float),
        ("fy", ctypes.c_float),
        ("fz", ctypes.c_float),
        ("tx", ctypes.c_float),
        ("ty", ctypes.c_float),
        ("tz", ctypes.c_float),
    ]

class FT_mod(ctypes.Union):
    _fields_ = [
        ("ft_array", FTarray_mod),
        ("ft_vect",  FTvect_mod),
    ]

    
