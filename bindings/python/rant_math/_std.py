"""The standard types. Rant's own when Rant is installed, so one class in a process, else the private
copy this wheel carries."""

try:
    from rant_types import *  # noqa: F401,F403
    from rant_types import ImageFormat, ScalarKind  # noqa: F401
except ImportError:
    from ._rant_types import *  # noqa: F401,F403
    from ._rant_types import ImageFormat, ScalarKind  # noqa: F401
