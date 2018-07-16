

ft_policy_map = {
    'Torque':     {'pos':0,'nb': 12, 'fmt':'h'*6,'fields':['rfx','rfy','rfz','rtx','rty','rtz']},
    'Mod_Torque': {'pos':1,'nb': 24, 'fmt':'i'*6,'fields':[ 'fx', 'fy', 'fz', 'tx', 'ty', 'tz']},
}


mc_policy_map = {
    # first byte policy --------------------------------
    'Position': {'pos':0, 'nb':4, 'fmt':'i'},
    'Velocity': {'pos':1, 'nb':2, 'fmt':'h'},
    'Torque':   {'pos':2, 'nb':2, 'fmt':'h'},
    'PID_out':  {'pos':3, 'nb':2, 'fmt':'h'},
    'PID_err':  {'pos':4, 'nb':4, 'fmt':'i'},
    'Current':  {'pos':5, 'nb':4, 'fmt':'i'},
    'Temp_DC':  {'pos':6, 'nb':4, 'fmt':'i'},
    'Timestamp':{'pos':7, 'nb':4, 'fmt':'i'},
    # second byte policy --------------------------------
    'Faults':          {'pos':8, 'nb':2, 'fmt':'B'*2, 'fields':['Faults_0','Faults_1']},
    'Ain_0':           {'pos':9, 'nb':2, 'fmt':'h'},
    'Ain_1':           {'pos':10,'nb':6, 'fmt':'h'*3, 'fields':['Ain_1','Ain_2','Ain_3']},
    'Link_pos':        {'pos':11,'nb':2, 'fmt':'h'},
    'Unaverage_speed': {'pos':12,'nb':2, 'fmt':'h'},
    'Motor_state':     {'pos':13,'nb':2, 'fmt':'h'},
    'Low_fltr_curr':   {'pos':14,'nb':2, 'fmt':'h'},
    'Link_deflection': {'pos':15,'nb':2, 'fmt':'h'},
    # first byte extra policy ---------------------------
    'Target_pos':     {'pos':16,'nb':4, 'fmt':'i'},
    'TempTarget_pos': {'pos':17,'nb':4, 'fmt':'i'},
    'Req_Target_pos': {'pos':18,'nb':4, 'fmt':'i'},
    
}


carm_policy_map = {
    'Position': {'pos':0, 'nb':4, 'fmt':'i'},
    'Velocity': {'pos':1, 'nb':2, 'fmt':'h'},
    'Torque':   {'pos':2, 'nb':2, 'fmt':'h'},
    'PID_out':  {'pos':3, 'nb':2, 'fmt':'h'},
    'PID_err':  {'pos':4, 'nb':4, 'fmt':'i'},
    'Current':  {'pos':5, 'nb':4, 'fmt':'i'},
    'TT_2':     {'pos':6, 'nb':4, 'fmt':'i'},
    'TT_3':     {'pos':7, 'nb':4, 'fmt':'i'},
    # ---------------------------------------
    'Faults':      {'pos':8, 'nb':2, 'fmt':'b'*2, 'fields':['Faults_0','Faults_1']},
    'Ain_0':       {'pos':9, 'nb':2, 'fmt':'h'},
    'Ain_1':       {'pos':10,'nb':6, 'fmt':'h'*3, 'fields':['Ain_1','Ain_2','Ain_3']},
    'Link_pos':     {'pos':11,'nb':2, 'fmt':'h'},
    'Quick_speed': {'pos':12,'nb':2, 'fmt':'h'},
    'Motor_state': {'pos':13,'nb':2, 'fmt':'h'},
    'Real_curr':   {'pos':14,'nb':2, 'fmt':'h'},
    'Rel_pos':     {'pos':15,'nb':2, 'fmt':'h'},
    # first byte extra policy ---------------------------
    'Target_pos':     {'pos':16,'nb':4, 'fmt':'i'},
    'TempTarget_pos': {'pos':17,'nb':4, 'fmt':'i'},
    'Req_Target_pos': {'pos':18,'nb':4, 'fmt':'i'},
    # second byte extra policy ---------------------------
    'Piezo_out':      {'pos':24,'nb':2, 'fmt':'h'},
    
}


bigLeg_policy_map = {
    'Position': {'pos':0, 'nb':4, 'fmt':'i'},
    'Velocity': {'pos':1, 'nb':2, 'fmt':'h'},
    'Torque':   {'pos':2, 'nb':2, 'fmt':'h'},
    'PID_out':  {'pos':3, 'nb':2, 'fmt':'h'},
    'PID_err':  {'pos':4, 'nb':4, 'fmt':'i'},
    'Current':  {'pos':5, 'nb':4, 'fmt':'i'},
    'Temp_DC':  {'pos':6, 'nb':4, 'fmt':'i'},
    'Tendon_tor':{'pos':7, 'nb':4, 'fmt':'i'},
    # ---------------------------------------
    'Faults':      {'pos':8, 'nb':2, 'fmt':'b'*2, 'fields':['Faults_0','Faults_1']},
    'Ain_0':       {'pos':9, 'nb':2, 'fmt':'h'},
    'Ain_1':       {'pos':10,'nb':6, 'fmt':'h'*3, 'fields':['Ain_1','Ain_2','Ain_3']},
    'Hip_pos':     {'pos':11,'nb':2, 'fmt':'h'},
    'Unavg_speed': {'pos':12,'nb':2, 'fmt':'h'},
    'Motor_state': {'pos':13,'nb':2, 'fmt':'h'},
    'Real_curr':   {'pos':14,'nb':2, 'fmt':'h'},
    'Height':      {'pos':15,'nb':2, 'fmt':'h'},
    # first byte extra policy ---------------------------
    'Target_pos':     {'pos':16,'nb':4, 'fmt':'i'},
    'TempTarget_pos': {'pos':17,'nb':4, 'fmt':'i'},
    'Req_Target_pos': {'pos':18,'nb':4, 'fmt':'i'},
    'Lin_enc_pos':    {'pos':19,'nb':4, 'fmt':'i'},
    'Lin_enc_raw':    {'pos':22,'nb':4, 'fmt':'i'},
    'Delta_tor':   {'pos':23,'nb':4, 'fmt':'i'},
    # second byte extra policy ---------------------------
    'Lin_enc_vel':    {'pos':28,'nb':2, 'fmt':'h'},
}