"""numpy and the standard types, for anyone who has numpy. Each is a view with no copy where it can be."""

from ._native import ErrorKind, MathError
from ._std import *  # noqa: F401,F403
from ._std import ImageFormat, ScalarKind

_CHANNELS = {
    ImageFormat.Mono8: ("uint8", 1), ImageFormat.Mono16: ("<u2", 1), ImageFormat.Rgb8: ("uint8", 3),
    ImageFormat.Rgba8: ("uint8", 4), ImageFormat.Bgr8: ("uint8", 3), ImageFormat.Monof32: ("<f4", 1),
}

_SCALARS = {
    ScalarKind.I8: "i1", ScalarKind.U8: "u1", ScalarKind.I16: "<i2", ScalarKind.U16: "<u2", ScalarKind.I32: "<i4",
    ScalarKind.U32: "<u4", ScalarKind.F32: "<f4", ScalarKind.F64: "<f8", ScalarKind.I64: "<i8", ScalarKind.U64: "<u8",
}


def image_to_numpy(image):
    """An Image as an array of rows by columns, with a third axis when it has channels. It shares
    image.data, so a writable image gives a writable array."""
    import numpy
    fmt = ImageFormat(image.format)
    if fmt not in _CHANNELS:
        raise MathError(ErrorKind.Format)
    kind, channels = _CHANNELS[fmt]
    item = numpy.dtype(kind).itemsize * channels
    stride = image.stride or image.width * item
    if len(image.data) < stride * image.height:
        raise MathError(ErrorKind.Format)
    shape = (image.height, image.width, channels) if channels > 1 else (image.height, image.width)
    strides = (stride, item, numpy.dtype(kind).itemsize) if channels > 1 else (stride, item)
    return numpy.ndarray(shape, dtype=kind, buffer=image.data, strides=strides)


def image_from_numpy(array, format=None):
    """An array of rows by columns as an Image with a copy of its pixels. Without a format, two axes of
    uint8 are Mono8, of uint16 Mono16 and of float32 Monof32, and three axes must say which they are."""
    import numpy
    array = numpy.asarray(array)
    if format is None:
        names = {("uint8", 2): ImageFormat.Mono8, ("uint16", 2): ImageFormat.Mono16, ("float32", 2): ImageFormat.Monof32}
        format = names.get((array.dtype.name, array.ndim))
        if format is None:
            raise MathError(ErrorKind.Argument)
    format = ImageFormat(format)
    if format not in _CHANNELS:
        raise MathError(ErrorKind.Format)
    kind, channels = _CHANNELS[format]
    if array.ndim != (3 if channels > 1 else 2) or (channels > 1 and array.shape[2] != channels):
        raise MathError(ErrorKind.Argument)
    data = bytearray(array.astype(kind).tobytes(order="C"))
    return Image(array.shape[1], array.shape[0], 0, format, data)


def cloud_to_numpy(cloud):
    """A PointCloud as a structured array with one named field per PointField. It shares cloud.data."""
    import numpy
    if not cloud.point_step or len(cloud.data) % cloud.point_step:
        raise MathError(ErrorKind.Format)
    names = [f.name for f in cloud.fields]
    formats = [(_SCALARS[ScalarKind(f.kind)], (f.count,)) if f.count > 1 else _SCALARS[ScalarKind(f.kind)] for f in cloud.fields]
    offsets = [f.offset for f in cloud.fields]
    dtype = numpy.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": cloud.point_step})
    return numpy.frombuffer(cloud.data, dtype=dtype)


def cloud_from_numpy(array):
    """A structured array as a PointCloud with a copy of its bytes and one PointField per name."""
    import numpy
    array = numpy.asarray(array)
    if array.dtype.names is None:
        raise MathError(ErrorKind.Argument)
    fields = []
    kinds = {numpy.dtype(v).newbyteorder("<"): k for k, v in _SCALARS.items()}
    for name in array.dtype.names:
        kind, offset = array.dtype.fields[name][:2]
        count = 1
        if kind.subdtype is not None:
            kind, shape = kind.subdtype
            count = int(numpy.prod(shape))
        fields.append(PointField(name, offset, kinds[kind.newbyteorder("<")], count))
    return PointCloud(Transform(), 0, fields, array.dtype.itemsize, False, bytearray(array.tobytes()))


def points_to_numpy(points):
    """A list of standard points or poses as an array of one row each."""
    import dataclasses
    import numpy

    def flat(v):
        out = []
        for f in dataclasses.fields(v):
            x = getattr(v, f.name)
            out += flat(x) if dataclasses.is_dataclass(x) else [x]
        return out
    return numpy.array([flat(p) for p in points], dtype="float64")


def points_from_numpy(array, kind=None):
    """An array of one row each as Double3 for three columns and Double2 for two, or kind for any other."""
    import dataclasses
    import numpy
    array = numpy.asarray(array, dtype="float64")
    if kind is None:
        kind = {2: Double2, 3: Double3}.get(array.shape[1] if array.ndim == 2 else 0)
        if kind is None:
            raise MathError(ErrorKind.Argument)

    def build(cls, row):
        args = []
        for f in dataclasses.fields(cls):
            inner = cls.__dataclass_fields__[f.name].default_factory
            if inner is not dataclasses.MISSING and dataclasses.is_dataclass(inner):
                value, row = build(inner, row)
                args.append(value)
            else:
                args.append(float(row[0]))
                row = row[1:]
        return cls(*args), row
    return [build(kind, row)[0] for row in array]
