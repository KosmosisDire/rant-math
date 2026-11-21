"""The shared library and the errors its results mean."""

import ctypes
import enum
import os
import sys


class ErrorKind(enum.IntEnum):
    """What a call that failed reports, the C result it came from."""
    Argument = -1    # a list too short, or a value out of range
    Degenerate = -2  # no single answer: a singular matrix, points on one line
    Space = -3       # never seen here, a binding always gives the room asked for
    Format = -4      # an image or cloud laid out in a way this call cannot read
    Memory = -5      # out of memory
    Outside = -6     # a point the call cannot map, such as one behind the camera


class MathError(Exception):
    """A call that could not answer. kind says why."""
    def __init__(self, kind):
        self.kind = ErrorKind(kind)
        super().__init__(TEXT[self.kind])


TEXT = {
    ErrorKind.Argument: "a list too short, or a value out of range",
    ErrorKind.Degenerate: "no single answer",
    ErrorKind.Space: "the output is too small",
    ErrorKind.Format: "an image or cloud laid out in a way this call cannot read",
    ErrorKind.Memory: "out of memory",
    ErrorKind.Outside: "a point the call cannot map",
}


def check(result):
    if result != 0:
        raise MathError(result)


def room(result):
    # The answer to asking a list's size: too small is what the question expects.
    if result != 0 and result != ErrorKind.Space:
        raise MathError(result)


def same_count(a, b):
    if a != b:
        raise MathError(ErrorKind.Argument)
    return a


def library_name():
    if sys.platform == "win32":
        return "rant_math.dll"
    if sys.platform == "darwin":
        return "librant_math.dylib"
    return "librant_math.so"


def library():
    # RANT_MATH_LIBRARY points a checkout's tests at a build. A wheel holds it beside this file.
    path = os.environ.get("RANT_MATH_LIBRARY") or os.path.join(os.path.dirname(os.path.abspath(__file__)), library_name())
    return ctypes.CDLL(path)
