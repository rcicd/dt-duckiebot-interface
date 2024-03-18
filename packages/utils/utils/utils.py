import os
from glob import glob

def map_recursive_files(dest, directory):
    return os.path.join(dest, directory), [y for x in os.walk(directory) for y in glob(os.path.join(x[0], '*'))]
