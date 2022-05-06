from typing import Iterable, Tuple, TypeVar, List
import glob
import os
import operator as op
from statistics import mean
from contextlib import contextmanager
import sys
import numpy as np


T = TypeVar('T')
# https://www.geeksforgeeks.org/break-list-chunks-size-n-python/
def chunkify(itr: Iterable[T], chunk_size: int) -> Iterable[Tuple[T]]:
    """ Split a list into chunks of a fixed size """
    itr = tuple(itr)
    for i in range(0, len(itr), chunk_size):
        yield itr[i: i + chunk_size]

def pluralize(num: int, string: str) -> str:
    """ Ex. given 5 and 'robot', return '5 robots'. """
    if num == 1:
        return f'{num} {string}'
    return f'{num} {string}s'

def delete_files(*,
                 recursive=True,
                 file: str = None,
                 files: List[str] = None,
                 file_pattern: str = None) -> None:
    inputs = (file, files, file_pattern)
    assert inputs.count(None) == len(inputs)-1, 'must provide exactly one of `files` or `file_patterh`'

    if file is not None:
        files = [file]
    elif file_pattern is not None:
        files = glob.glob(file_pattern, recursive=recursive)

    for file in files:
        try:
            os.remove(file)
        except OSError:
            print(f'error: OSError while deleting file "{file}"')

def add_tup(t1: tuple, t2: tuple) -> tuple:
    """ Preform an elementwise sum. """
    return tuple(map(op.add, t1, t2))

def mult_tup(t1: tuple, t2: tuple) -> tuple:
    """ Perform elementwise multiplication. """
    return tuple(map(op.mul, t1, t2))

def mean_tup(*ts: tuple) -> tuple:
    """ Perform an elementwise mean. """
    return tuple(map(mean, zip(*ts)))

def dist(v1, v2):
    """ Reimplement math.dist for Python 3.6 """
    assert len(v1) == len(v2)
    v1 = np.array(v1)
    v2 = np.array(v2)
    return np.linalg.norm(v1 - v2)

class LiteralType:
    """ Reimplement typing.Literal for Python 3.6 """
    def __getitem__(self, key):
        return str
Literal = LiteralType()


''' Doesn't work \/ '''
# class HideOutput(object):
#     '''
#     A context manager that block stdout for its scope, usage:
#
#     with HideOutput():
#         os.system('ls -l')
#     '''
#
#     def __init__(self, *args, **kw):
#         sys.stdout.flush()
#         self._origstdout = sys.stdout
#         self._oldstdout_fno = os.dup(sys.stdout.fileno())
#         self._devnull = os.open(os.devnull, os.O_WRONLY)
#
#     def __enter__(self):
#         self._newstdout = os.dup(1)
#         os.dup2(self._devnull, 1)
#         os.close(self._devnull)
#         sys.stdout = os.fdopen(self._newstdout, 'w')
#
#     def __exit__(self, exc_type, exc_val, exc_tb):
#         sys.stdout = self._origstdout
#         sys.stdout.flush()
#         os.dup2(self._oldstdout_fno, 1)
