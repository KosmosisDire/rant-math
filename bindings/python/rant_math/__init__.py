"""Rant Math for Python: every call of the C library as rant_math.<name> on the standard types.

A failure raises MathError. A list is any iterable of standard types or a numpy array, and numpy in
gives numpy out, with no copy of what goes in."""

from ._api import *  # noqa: F401,F403
from ._api import __all__ as _calls
from ._convert import cloud_from_numpy, cloud_to_numpy, image_from_numpy, image_to_numpy, points_from_numpy, points_to_numpy
from ._std import *  # noqa: F401,F403

__all__ = list(_calls) + ["cloud_from_numpy", "cloud_to_numpy", "image_from_numpy", "image_to_numpy", "points_from_numpy", "points_to_numpy"]
