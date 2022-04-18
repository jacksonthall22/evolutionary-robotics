from typing import Iterable, Tuple, TypeVar, List
import glob
import os
from contextlib import contextmanager
import sys

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
            print('error: OSError while deleting file "{file}"')


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
