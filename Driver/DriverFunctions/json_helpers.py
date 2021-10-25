import re
import os
from globals import *

def get_numbers_from_filename(filename):
    val = 0
    for s in re.findall(r'\b\d+\b', filename):
        val = int(s)
    return val

def get_max_number_in_filename(filedirectory):
    max_val = 0
    for filename in os.listdir(filedirectory):
        val = get_numbers_from_filename(filename)
        if val > max_val:
            max_val = val
    return max_val

def get_new_json_filename(controllername, filedirectory = JSON_PATH):
    previd = get_max_number_in_filename(filedirectory)
    id = previd + 1
    id_str = str(id)
    new_filename = filedirectory + 'control' + '_' + controllername + ' - ' + id_str + '.json'
    return new_filename
